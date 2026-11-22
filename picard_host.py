#!/usr/bin/env python3
"""PIC16F72 programmer — host side (PicArd Revised Edition).

Talks to the picard_revised Arduino sketch (Uno/Nano on Nelson Bittencourt's
PicArd shield circuit) over serial and runs the full DS39588A programming
flow: device-ID check, chip erase, blank check, program, verify. Hex parsing
and every read/verify routine are unchanged from pic72_host.py; the only
additions are the boost-converter calibration commands at the bottom.

Verify reprograms any word that reads back wrong and checks again (up to
REPAIR_PASSES rounds); flash can only clear bits, so a word that was only
partly written can be completed without erasing the chip.

Requires:  pip install pyserial

The Arduino auto-resets when the port opens; this script waits ~2 s for
the bootloader to time out before talking to the sketch.

Examples:
    python picard_host.py --port COM5 --id
    python picard_host.py --port COM5 --erase
    python picard_host.py --port COM5 --hex firmware.hex
    python picard_host.py --port COM5 --verify firmware.hex
    python picard_host.py --port COM5 --dump dump.txt
    python picard_host.py --port COM5 --boost
    python picard_host.py --port COM5 --boost-test
    python picard_host.py --port COM5 --set-target 950
    python picard_host.py --port COM5 --hold 30
    python picard_host.py --port COM5 --load-test 40

Boost calibration (only needed if VPP on MCLR is outside 12.75-13.25V):
    python picard_host.py --port COM5 --boost
      -> enters programming mode, holds VPP on the PIC, prints the A0
         feedback reading; put a meter on the PIC's MCLR pin, type the
         voltage you measure, and it prints the corrected target.
    python picard_host.py --port COM5 --set-target <N>
      -> stores that target in the Arduino's EEPROM.
"""

import argparse
import os
import time

import serial

DEVID_PIC16F72 = 5
BLANK = 0x3FFF
CODE_START, CODE_END = 0x0000, 0x07FF
CFG_START, CFG_END = 0x2000, 0x2007
DEVID_ADDR = 0x2006           # written at the factory — never programmed
CONFIG_WORD_ADDR = 0x2007
CODE_WORDS = CODE_END - CODE_START + 1
# DS39588A Register 3-1: config bits 13-7 and 5 are unimplemented and read
# back as '1', so only 0x005F is comparable (same mask as Table 4-1 CFWD).
CONFIG_MASK = 0x005F
HEX_CHARS = '0123456789abcdefABCDEF'

VPP_TARGET_VOLTS = 13.0       # centre of the 12.75-13.25V window
DUTY_MAX = 560                # matches the sketch's duty ceiling
REPAIR_PASSES = 3             # verify/reprogram rounds before giving up


class Programmer:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=5, write_timeout=5)
        time.sleep(2.2)                 # Arduino auto-reset on port open
        self.ser.reset_input_buffer()
        r = self._send('?')
        if r != 'P72':
            raise RuntimeError(f"Unexpected handshake response: {r!r} "
                               "(is the picard_revised sketch loaded?)")

    def _send(self, cmd):
        self.ser.write((cmd + '\n').encode('ascii'))
        line = self.ser.readline().decode('ascii').strip()
        if not line:
            raise RuntimeError(f"Arduino did not respond to {cmd!r}")
        if line == 'EV':
            raise RuntimeError(
                "VPP regulation failed - the boost converter did not reach its "
                "target, so programming mode was not entered. Run --boost-test "
                "to see the plateau and duty, then check Q4/L1/D2 and the A0 "
                "feedback divider.")
        if line == 'ER':
            raise RuntimeError(f"Arduino rejected {cmd!r}")
        return line

    def _send_data(self, cmd):
        line = self._send(cmd)
        if not line.startswith('D'):
            raise RuntimeError(f"Expected data response to {cmd!r}, got {line!r}")
        return int(line[1:], 16)

    def close(self):
        try:
            self._send('X')             # make sure VPP is dropped
        except Exception:
            pass
        self.ser.close()

    # --- protocol primitives ---
    def enter(self):        self._send('E')
    def exit_prog(self):    self._send('X')
    def erase(self):        self._send('A')
    def load_config(self):  self._send('C')
    def inc(self):          self._send('N')

    def prog_word(self, w):
        if not 0 <= w <= 0x3FFF:
            raise ValueError(f"word {w:#06x} exceeds 14 bits")
        self._send(f'P{w:04X}')

    def read_word(self):
        return self._send_data('R')

    def read_id(self):
        word = self._send_data('I')
        return word >> 5, word & 0x1F, word

    def read_block(self, count):
        last = None
        for _ in range(3):
            line = self._send(f'G{count:04X}')
            if len(line) == 4 * count and all(c in HEX_CHARS for c in line):
                return [int(line[i:i + 4], 16) for i in range(0, len(line), 4)]
            last = f"{len(line)} chars for {count} words"
            # A hiccup can truncate the stream with the Arduino's address
            # counter mid-block. Drain, then re-enter programming mode to
            # rewind PC to 0x0000 before retrying.
            time.sleep(2.0)             # let any in-flight stream finish
            self.ser.reset_input_buffer()
            self._send('E')
        raise RuntimeError(f"Short block read: {last} (3 attempts)")

    def read_at(self, addr, tries=3):
        """Seek to addr and read that word `tries` times (R does not increment)."""
        for attempt in range(3):
            self._send('E')                       # PC = 0x0000
            if not addr:
                break
            line = self._send(f'G{addr:04X}')     # stream-discard addr words
            if len(line) == 4 * addr and all(c in HEX_CHARS for c in line):
                break                             # PC = addr, stream in sync
            if attempt == 2:
                raise RuntimeError(f"read stream corrupted while seeking to {addr:#06x}")
            time.sleep(2.0)                       # drain, then resync from PC=0
            self.ser.reset_input_buffer()
        return [self.read_word() for _ in range(tries)]


def parse_hex(path):
    """Parse Intel HEX into {addr: 14-bit word} dicts for code and config space."""
    code, config = {}, {}
    with open(path) as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.strip()
            if not line:
                continue
            if line[0] != ':':
                raise ValueError(f"line {lineno}: missing ':' start code")
            try:
                nb = int(line[1:3], 16)
                hi = int(line[3:5], 16)
                lo = int(line[5:7], 16)
                dtype = int(line[7:9], 16)
                payload = line[9:9 + 2 * nb]
                csum = int(line[9 + 2 * nb:11 + 2 * nb], 16)
            except ValueError:
                raise ValueError(f"line {lineno}: malformed record")
            if len(payload) != 2 * nb:
                raise ValueError(f"line {lineno}: record length mismatch")
            total = (nb + hi + lo + dtype +
                     sum(int(payload[i:i + 2], 16) for i in range(0, 2 * nb, 2))) & 0xFF
            if ((-total) & 0xFF) != csum:
                raise ValueError(f"line {lineno}: checksum mismatch")
            if dtype == 1:              # EOF
                break
            if dtype != 0:
                raise ValueError(f"line {lineno}: record type {dtype} not supported")
            if nb % 2:
                raise ValueError(f"line {lineno}: odd byte count {nb} — not word-aligned")
            byte_addr = (hi << 8) | lo
            if byte_addr % 2:
                raise ValueError(f"line {lineno}: odd byte address {byte_addr:#06x}")
            base = byte_addr // 2
            for i in range(nb // 2):
                word = (int(payload[4 * i + 2:4 * i + 4], 16) << 8) | int(payload[4 * i:4 * i + 2], 16)
                if word > 0x3FFF:
                    raise ValueError(f"line {lineno}: word {word:#06x} exceeds 14 bits")
                wa = base + i
                if CODE_START <= wa <= CODE_END:
                    code[wa] = word
                elif CFG_START <= wa <= CFG_END:
                    config[wa] = word
                else:
                    raise ValueError(f"line {lineno}: address {wa:#06x} outside PIC16F72 memory")
    return code, config


def check_device(p):
    ids = [p.read_id() for _ in range(3)]        # majority vote vs read glitches
    devid, rev, word = max(set(ids), key=ids.count)
    if devid != DEVID_PIC16F72:
        raise RuntimeError(
            f"Device ID {devid} (raw word {word:#06x}) — expected {DEVID_PIC16F72} "
            "for PIC16F72. Check wiring, VPP (12.75-13.25V) and VDD (4.75-5.25V); "
            "run --boost if VPP needs calibrating.")
    print(f"PIC16F72 detected, silicon rev {rev} (ID word {word:#06x})")
    return devid, rev


def blank_check(p):
    p.enter()
    words = p.read_block(CODE_WORDS)
    nonblank = [a for a, w in enumerate(words) if w != BLANK]
    if nonblank:
        real = []
        for a in nonblank:
            reads = p.read_at(a)
            if sum(v == BLANK for v in reads) >= 2:
                continue          # corrupted in transit, not a real bad word
            real.append(a)
        if real:
            shown = ', '.join(f"{a:04X}" for a in real[:8])
            raise RuntimeError(f"Blank check failed at {shown}"
                               f"{' ...' if len(real) > 8 else ''} — erase may have failed")
        print(f"    note: {len(nonblank)} word(s) glitched during the blank-check "
              "read but read back blank")
    print("Chip erased, blank check passed")


def do_id(p):
    check_device(p)


def do_erase(p):
    check_device(p)
    t0 = time.time()
    p.enter()
    p.erase()
    blank_check(p)
    print(f"Erase completed in {time.time() - t0:.1f}s")


def do_dump(p, path):
    check_device(p)
    p.enter()
    code = p.read_block(CODE_WORDS)
    p.enter()
    second = p.read_block(CODE_WORDS)      # two passes, then majority-resolve diffs
    diffs = [a for a in range(CODE_WORDS) if code[a] != second[a]]
    for a in diffs:
        reads = p.read_at(a) + [code[a], second[a]]
        code[a] = max(set(reads), key=reads.count)
    if diffs:
        print(f"    note: {len(diffs)} word(s) differed between read passes; "
              "resolved by majority re-read")
    p.load_config()
    cfg, cur = [], CFG_START
    for addr in range(CFG_START, CFG_END + 1):     # R does not increment
        for _ in range(addr - cur):
            p.inc()
        reads = [p.read_word() for _ in range(3)]
        cfg.append(max(set(reads), key=reads.count))
        cur = addr
    with open(path, 'w') as f:
        f.write("addr  word\n")
        for a, w in enumerate(code):
            f.write(f"{a:04X}  {w:04X}\n")
        for a, w in zip(range(CFG_START, CFG_END + 1), cfg):
            f.write(f"{a:04X}  {w:04X}\n")
    print(f"Dumped {len(code)} program words + {len(cfg)} config words to {path}")


def triage_mismatches(p, code, mism):
    """Split verify mismatches into read glitches, real errors and flaky reads.

    A word that reads correctly in a majority of fresh reads was corrupted in
    transit; the chip itself is fine. Words that read the same wrong value every
    time really are wrong (genuine); words that never settle mean a flaky link.

    Returns (glitches, genuine, unstable), where genuine is a list of
    (addr, readback) tuples and unstable is a list of (addr, reads).
    """
    def expect(a):
        return code.get(a, BLANK)

    glitches = []
    if len(mism) > 64:                          # too many for word-by-word triage
        p.enter()
        second = p.read_block(CODE_WORDS)       # one more full pass
        glitches += [a for a in mism if second[a] == expect(a)]
        mism = [a for a in mism if second[a] != expect(a)]
        if len(mism) > 64:
            raise RuntimeError("bulk reads are too corrupted to verify - fix the "
                               "wiring before trying again")

    genuine, unstable = [], []
    for a in mism:
        reads = p.read_at(a)
        if sum(v == expect(a) for v in reads) >= 2:
            glitches.append(a)                  # majority correct: transit glitch
        elif len(set(reads)) == 1:
            genuine.append((a, reads[0]))
        else:
            unstable.append((a, reads))

    if genuine:
        if all(v == 0 for _, v in genuine):
            print("    note: these words read back as 0x0000 - if this hex enables "
                  "code protection (CP=0), program memory reads as zeros and "
                  "verify cannot work by design")
        erased = [a for a, v in genuine if (v & ~expect(a)) & 0x3FFF]
        latched_low = [a for a, v in genuine if (~v & expect(a)) & 0x3FFF]
        if erased and not latched_low:
            print("    note: every mismatch is an erased cell that should have been "
                  "programmed (readback has only extra 1s) - the word reached the "
                  "chip intact and the cell did not charge")
        if latched_low:
            print(f"    note: {len(latched_low)} of {len(genuine)} word(s) have 0s "
                  "where the hex has 1s - a data bit was latched low while the word "
                  "was shifted into the chip (PGC/PGD noise or a corrupted 'P' "
                  "command, not a VPP/VDD level problem)")
    return glitches, genuine, unstable


def report_vpp_span(p):
    """Report the boost's extremes over the program phase ('T' command)."""
    line = p._send('T')
    if not line.startswith('D'):
        raise RuntimeError(f"Unexpected response to T: {line!r}")
    mn, mx, dp = (int(x, 16) for x in line[1:].split())
    target = p._send_data('W')
    print(f"    VPP while programming: A0 {mn}-{mx} (target {target}), "
          f"duty peak {dp}/{DUTY_MAX}")
    if target:
        dip = (target - mn) * VPP_TARGET_VOLTS / target
        print(f"      deepest dip: {dip:.2f}V below target "
              f"({estimate_volts(mn, target):.2f}V at the pins)")
    if mn < target - 30:
        print("    the rail sagged well below target during the run - that is what "
              "leaves cells unprogrammed; add bulk capacitance on the VPP rail "
              "(C1 is only 47uF) and check the 5V supply")
    if dp >= DUTY_MAX - 1:
        print("    the switch sat at the duty ceiling, so the rail has no headroom "
              "under load - L1/R10/D2 or a weak 5V input")


def estimate_volts(count, target):
    """VPP volts from an A0 count, using the calibrated target as 13.0V."""
    return count * VPP_TARGET_VOLTS / target


def do_load_test(p, n):
    """Worst-case programming load: n all-zero words at the real cadence."""
    p.enter()
    line = p._send(f'Q{n:04X}')
    mn, mx, dp, ms = (int(x, 16) for x in line[1:].split())
    target = p._send_data('W')
    print(f"{n} all-zero words programmed in {ms} ms (worst-case pulse current)")
    print(f"VPP during pulses: A0 {mn}-{mx} (target {target}), "
          f"duty peak {dp}/{DUTY_MAX}")
    if target:
        print(f"  target {VPP_TARGET_VOLTS:.2f}V, low point "
              f"{estimate_volts(mn, target):.2f}V, dip "
              f"{(target - mn) * VPP_TARGET_VOLTS / target:.2f}V")
        if mn < target - 12:
            print(f"  the rail drops below the 12.75V minimum during the pulse:")
            print("  47uF (C1) cannot hold 13V through a 3ms pulse - add a")
            print("  470uF/25V electrolytic across C1, as close to Q2 as possible.")
        else:
            print("  the rail holds through the pulse - this board can program.")
    print("The chip now holds zeros - the next --hex run erases it first.")


def do_hold(p, secs):
    """Keep VPP applied so the rails can be metered under real load."""
    adc = p._send_data('V')            # enters programming mode, VPP stays on
    target = p._send_data('W')
    print(f"Programming mode held: A0 {adc} counts (target {target})")
    print("Meter VPP at PIC pin 1 (MCLR) and VDD at pin 20 now - both must hold")
    print("VPP 12.75-13.25V and VDD 4.75-5.25V.")
    t0 = time.time()
    try:
        while time.time() - t0 < secs:
            time.sleep(1.0)
            a, d = (int(x, 16) for x in p._send('S')[1:].split())
            print(f"  {int(time.time() - t0):3d}s  A0 {a:4d}  duty {d:3d}")
    finally:
        mn, mx, dp = (int(x, 16) for x in p._send('T')[1:].split())
        print(f"Over the window: A0 {mn}-{mx}, duty peak {dp}/{DUTY_MAX}")


def do_full(p, hexfile):
    code, config = parse_hex(hexfile)
    if DEVID_ADDR in config:
        print(f"note: hex contains a device-ID word at {DEVID_ADDR:#06x} — not programmed")
        config.pop(DEVID_ADDR)
    print(f"[1] {hexfile}: {len(code)} program words, {len(config)} config/ID words")
    if CONFIG_WORD_ADDR not in config:
        print("    WARNING: no configuration word at 0x2007 in the hex file.")
        print("    Unprogrammed config (0x3FFF) = FOSC:RC, WDT:on, BOR:on, PWRT:off.")

    print("[2] ", end='')
    check_device(p)

    t0 = time.time()
    p.enter()
    p.erase()
    blank_check(p)
    print(f"      erase+blank {time.time() - t0:.1f}s")

    t0 = time.time()
    p.enter()                          # PC = 0x0000
    cur = CODE_START
    for addr in sorted(code):
        for _ in range(addr - cur):
            p.inc()
        p.prog_word(code[addr])
        cur = addr + 1                 # P increments after programming
    print(f"[3] Programmed {len(code)} words in {time.time() - t0:.1f}s")

    if config:
        t0 = time.time()
        p.load_config()                # PC = 0x2000
        cur = CFG_START
        for addr in sorted(config):
            for _ in range(addr - cur):
                p.inc()
            p.prog_word(config[addr])
            cur = addr + 1
        addrs = ', '.join(f"{a:04X}" for a in sorted(config))
        print(f"[4] Programmed config/ID words ({addrs}) in {time.time() - t0:.1f}s")

    report_vpp_span(p)

    t0 = time.time()
    passes = 0
    history = []
    while True:
        passes += 1
        p.enter()
        words = p.read_block(CODE_WORDS)
        mism = [a for a in range(CODE_START, CODE_END + 1)
                if words[a] != code.get(a, BLANK)]
        if mism and all(w == 0 for w in words):
            raise RuntimeError("all 2048 words read back 0x0000 - either programming "
                               "failed or this hex enables code protection (CP=0), "
                               "which makes program memory unverifiable by design")
        if not mism:
            break
        glitches, genuine, unstable = triage_mismatches(p, code, mism)
        if glitches:
            shown = ', '.join(f"{a:#06x}" for a in glitches[:8])
            more = f" (+{len(glitches) - 8} more)" if len(glitches) > 8 else ''
            print(f"    note: {len(glitches)} word(s) were corrupted in transit during "
                  f"the bulk read ({shown}{more}) but read back correctly - the chip "
                  "contents are good")
        if unstable:
            shown = ', '.join(f"{a:#06x}: {' '.join(format(v, '04X') for v in r)}"
                              for a, r in unstable[:4])
            raise RuntimeError(f"Verify failed - repeated reads of the same word keep "
                               f"changing ({shown}); the connection is unstable - "
                               "reseat all jumpers and shorten the PGC/PGD wiring")
        if not genuine:
            break
        retry = [a for a, _ in genuine if a in code]
        if len(retry) != len(genuine):
            lost = [a for a, _ in genuine if a not in code]
            raise RuntimeError("Verify failed at addresses that hold no data in the "
                               f"hex ({', '.join(f'{a:#06x}' for a in lost[:4])}); "
                               "those words cannot be repaired by reprogramming")
        history.append(sorted(retry))
        shown = ', '.join(f"{a:#06x} {code[a]:04X}->{v:04X}"
                          for a, v in sorted(genuine)[:6])
        more = f" (+{len(retry) - 6} more)" if len(retry) > 6 else ''
        if passes >= REPAIR_PASSES:
            latched = [a for a, v in genuine if (~v & code[a]) & 0x3FFF]
            hint = ("a bit now reads 0 where the hex has 1, and flash cannot clear "
                    "a programmed bit back to 1 - this chip needs a full erase, so "
                    "run --hex again" if latched else
                    "the same cells keep failing, so VPP or VDD really is out of "
                    "spec during the pulse - meter VDD at the PIC's pin 20 during "
                    "--load-test")
            raise RuntimeError(f"Verify failed - {len(retry)} word(s) still wrong after "
                               f"{passes} program passes ({shown}{more}); {hint}")
        print(f"    {len(retry)} word(s) did not take on pass {passes} ({shown}{more}) "
              f"- reprogramming just those (pass {passes + 1}/{REPAIR_PASSES})")
        p.enter()
        cur = CODE_START
        for addr in sorted(retry):
            for _ in range(addr - cur):
                p.inc()
            p.prog_word(code[addr])
            cur = addr + 1
    if history:
        sizes = ', '.join(str(len(h)) for h in history)
        repeat = set(history[0]).intersection(*history[1:]) if history[1:] else set()
        print(f"    note: extra program pass(es) for {sizes} word(s); "
              f"{len(repeat)} address(es) failed more than once"
              + (f" ({', '.join(f'{a:#06x}' for a in sorted(repeat)[:6])})"
                 if repeat else ""))
        print("    (those words programmed on retry, so the first-pass failures were "
              "transient - they are a margin problem, not a dead cell)")
    if config:
        p.load_config()
        cur = CFG_START
        for addr in sorted(config):
            for _ in range(addr - cur):
                p.inc()
            reads = [p.read_word() for _ in range(3)]     # R does not increment
            expect = config[addr] & CONFIG_MASK
            good = [r for r in reads if (r & CONFIG_MASK) == expect]
            if not good:
                raise RuntimeError(f"Config verify failed at {addr:#06x}: "
                                   f"wrote {config[addr]:#06x}, read {reads[0]:#06x}")
            if good[0] != config[addr]:
                print(f"    note: {addr:#06x} reads {good[0]:#06x} - unimplemented "
                      f"bits read as 1s; implemented bits match (mask {CONFIG_MASK:#06x})")
            cur = addr
    print(f"[5] Verify passed in {time.time() - t0:.1f}s")

    p.exit_prog()
    print("Done. Remove VDD/MCLR and the chip is ready to run.")


def do_verify(p, hexfile):
    """Read the chip and compare it with a hex file (no erase, no programming)."""
    code, config = parse_hex(hexfile)
    if DEVID_ADDR in config:
        config.pop(DEVID_ADDR)
    check_device(p)
    p.enter()
    words = p.read_block(CODE_WORDS)
    mism = [a for a in range(CODE_START, CODE_END + 1)
            if words[a] != code.get(a, BLANK)]
    bad = []
    if mism:
        glitches, genuine, unstable = triage_mismatches(p, code, mism)
        if glitches:
            shown = ', '.join(f"{a:#06x}" for a in glitches[:8])
            more = f" (+{len(glitches) - 8} more)" if len(glitches) > 8 else ''
            print(f"    note: {len(glitches)} word(s) misread in transit "
                  f"({shown}{more}) but read back correctly")
        if unstable:
            raise RuntimeError("reads of the same word keep changing - the "
                               "connection is unstable; reseat the jumpers")
        bad = genuine

    cfg_bad = []
    if config:
        p.load_config()
        cur = CFG_START
        for addr in sorted(config):
            for _ in range(addr - cur):
                p.inc()
            reads = [p.read_word() for _ in range(3)]     # R does not increment
            expect = config[addr] & CONFIG_MASK
            if not any((r & CONFIG_MASK) == expect for r in reads):
                cfg_bad.append((addr, expect, reads[0]))
            cur = addr

    if not bad and not cfg_bad:
        print(f"MATCH - chip holds all {len(code)} program words and "
              f"{len(config)} config word(s) from {os.path.basename(hexfile)}")
        return True

    print("MISMATCH")
    if bad:
        print(f"    {len(bad)} program word(s) differ:")
        for a, v in bad[:16]:
            print(f"      {a:#06x}  hex {code[a]:04X}  chip {v:04X}  "
                  f"bits off {((v ^ code[a]) & 0x3FFF):04X}")
        if len(bad) > 16:
            print(f"      ... {len(bad) - 16} more")
    if cfg_bad:
        print("    configuration word(s) differ:")
        for addr, expect, got in cfg_bad:
            print(f"      {addr:#06x}  hex {expect:04X} (masked)  chip {got:04X}")
    return False


def do_boost(p):
    """Hold VPP on MCLR, report the raw A0 feedback reading for calibration."""
    adc = p._send_data('V')            # enters programming mode, VPP stays on
    target = p._send_data('W')
    print(f"A0 feedback: {adc} counts (target {target})")
    print("VPP is on the PIC's MCLR pin right now.")
    print("Measure it with a meter (12.75-13.25V is the spec window).")
    print("Two minutes, then the Arduino drops VPP on its own.")
    raw = input("Enter the voltage you measured (or press Enter to skip): ").strip()
    if not raw:
        print("Skipped - no changes made.")
        return
    try:
        volts = float(raw.replace(',', '.'))
    except ValueError:
        print(f"Not a number: {raw!r} - no changes made.")
        return
    if not 5.0 <= volts <= 20.0:
        print(f"{volts}V is not a plausible VPP reading - no changes made.")
        return
    new = round(adc * VPP_TARGET_VOLTS / volts)
    new = max(300, min(1023, new))
    print(f"Measured {volts:.2f}V at ADC {adc}.")
    print(f"Suggested target for {VPP_TARGET_VOLTS:.1f}V: {new}")
    print(f"Save it with:  python {os.path.basename(__file__)} "
          f"--port {p.ser.port} --set-target {new}")


def do_boost_test(p):
    """Run the boost with no load and report where it plateaus."""
    line = p._send('B')
    if not line.startswith('D'):
        raise RuntimeError(f"Unexpected boost-test response: {line!r}")
    adc, duty, mn, mx = (int(x, 16) for x in line[1:].split())
    target = p._send_data('W')
    print(f"A0 feedback: {adc} counts  (target {target})")
    print(f"PWM duty:    {duty}/800 = {duty / 8:.1f}%")
    print(f"Last 400 ms: min {mn}, max {mx}  (swing {mx - mn} counts)")
    if mx - mn > 40:
        print("The loop is oscillating - the rail is not holding still.")
    if adc < 200 and duty >= DUTY_MAX - 1:
        print("A0 sits near zero while the switch is at maximum duty: the feedback")
        print("node is not reaching the Arduino. Check R2/R3, the A0 wire, and D1")
        print("(a reversed zener clamps A0 to ~0.7V).")
    elif adc >= 1015:
        print("A0 is pinned at the top of the range (zener clamp / divider swap).")
    elif duty >= DUTY_MAX - 1:
        print("Duty is pegged at the ceiling and it still cannot reach target:")
        print("check L1 (should be 2.2mH), D2, R10 56R, and Q4.")
    elif adc < target - 40:
        print("The loop stopped short of target with duty to spare - the reading")
        print("is stuck; check the A0 connection.")
    else:
        print("Boost reached its target.")


def do_set_target(p, value):
    if not 300 <= value <= 1023:
        raise ValueError("target must be 300-1023 ADC counts (about 4.5-14V at A0)")
    p._send(f'V{value:04X}')
    print(f"Boost target set to {value} counts and saved to EEPROM.")


def main():
    ap = argparse.ArgumentParser(
        description="PIC16F72 programmer host (PicArd Revised Edition Arduino ICSP engine)")
    ap.add_argument('--port', required=True, help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument('--hex', help="Intel HEX file: full erase/blank/program/verify run")
    ap.add_argument('--verify', metavar='FILE',
                    help="read the chip and compare it with a hex file (no erase)")
    ap.add_argument('--erase', action='store_true', help="chip erase + blank check")
    ap.add_argument('--id', action='store_true', help="read device ID and exit")
    ap.add_argument('--dump', metavar='FILE', help="dump full memory to a text file")
    ap.add_argument('--boost', action='store_true',
                    help="hold VPP on and report the A0 feedback reading (calibration)")
    ap.add_argument('--boost-test', action='store_true',
                    help="run the boost with no load and report plateau/duty/oscillation")
    ap.add_argument('--set-target', type=int, metavar='N',
                    help="set the boost ADC target (300-1023) and save it")
    ap.add_argument('--hold', nargs='?', type=int, const=20, metavar='SECS',
                    help="hold VPP/VDD on the chip for SECS (default 20) so the "
                         "rails can be metered under load")
    ap.add_argument('--load-test', nargs='?', type=int, const=40, metavar='N',
                    help="program N all-zero words (default 40) and report how far "
                         "VPP dips under worst-case pulse current")
    args = ap.parse_args()

    if not (args.hex or args.verify or args.erase or args.id or args.dump
            or args.boost or args.boost_test or args.set_target is not None
            or args.hold is not None or args.load_test is not None):
        ap.error("nothing to do: give --hex, --verify, --erase, --id, --dump, "
                 "--boost, --boost-test, --set-target, --hold, or --load-test")
    if args.hex and args.verify:
        ap.error("--hex already verifies; use --verify on its own to check a chip")

    p = Programmer(args.port)
    try:
        if args.id:
            do_id(p)
        if args.dump:
            do_dump(p, args.dump)
        if args.erase:
            do_erase(p)
        if args.verify:
            do_verify(p, args.verify)
        if args.hex:
            do_full(p, args.hex)
        if args.boost:
            do_boost(p)
        if args.boost_test:
            do_boost_test(p)
        if args.set_target is not None:
            do_set_target(p, args.set_target)
        if args.hold is not None:
            do_hold(p, args.hold)
        if args.load_test is not None:
            do_load_test(p, args.load_test)
    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        p.close()


if __name__ == '__main__':
    main()
