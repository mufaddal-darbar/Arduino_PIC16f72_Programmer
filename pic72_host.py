#!/usr/bin/env python3
"""PIC16F72 programmer — host side.

Talks to the pic16f72_icsp Arduino sketch (Uno/Nano) over serial and runs
the full DS39588A programming flow: device-ID check, chip erase, blank
check, program, verify. Hex parsing ported from Tahmid Mahbub's prog72.py.

Requires:  pip install pyserial

The Arduino auto-resets when the port opens; this script waits ~2 s for
the bootloader to time out before talking to the sketch.

Examples:
    python pic72_host.py --port COM5 --id
    python pic72_host.py --port COM5 --erase
    python pic72_host.py --port COM5 --hex firmware.hex
    python pic72_host.py --port COM5 --dump dump.txt
"""

import argparse
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


class Programmer:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=5, write_timeout=5)
        time.sleep(2.2)                 # Arduino auto-reset on port open
        self.ser.reset_input_buffer()
        r = self._send('?')
        if r != 'P72':
            raise RuntimeError(f"Unexpected handshake response: {r!r} "
                               "(is the pic16f72_icsp sketch loaded?)")

    def _send(self, cmd):
        self.ser.write((cmd + '\n').encode('ascii'))
        line = self.ser.readline().decode('ascii').strip()
        if not line:
            raise RuntimeError(f"Arduino did not respond to {cmd!r}")
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
            "for PIC16F72. Check wiring, VPP (12.75-13.25V) and VDD (4.75-5.25V).")
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


def resolve_mismatches(p, code, mism):
    """Triage verify mismatches into in-transit read glitches vs real errors.

    A word that reads correctly in a majority of fresh reads was corrupted in
    transit; the chip itself is fine. Raises for words that are consistently
    wrong (glitch during programming) or unstable (flaky connection). Returns
    the list of glitch addresses.
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
        shown = ', '.join(f"{a:#06x} {expect(a):04X}->{v:04X}" for a, v in genuine[:4])
        more = f" (+{len(genuine) - 4} more)" if len(genuine) > 4 else ''
        raise RuntimeError(f"Verify failed - {len(genuine)} word(s) read back "
                           f"consistently wrong ({shown}{more}); the corruption "
                           "happened while the chip was being programmed")
    if unstable:
        shown = ', '.join(f"{a:#06x}: {' '.join(format(v, '04X') for v in r)}"
                          for a, r in unstable[:4])
        raise RuntimeError(f"Verify failed - repeated reads of the same word keep "
                           f"changing ({shown}); the connection is unstable - "
                           "reseat all jumpers and shorten the PGC/PGD wiring")
    return glitches


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

    t0 = time.time()
    p.enter()
    words = p.read_block(CODE_WORDS)
    mism = [a for a in range(CODE_START, CODE_END + 1)
            if words[a] != code.get(a, BLANK)]
    if mism and all(w == 0 for w in words):
        raise RuntimeError("all 2048 words read back 0x0000 - either programming "
                           "failed or this hex enables code protection (CP=0), "
                           "which makes program memory unverifiable by design")
    if mism:
        glitches = resolve_mismatches(p, code, mism)
        shown = ', '.join(f"{a:#06x}" for a in glitches[:8])
        more = f" (+{len(glitches) - 8} more)" if len(glitches) > 8 else ''
        print(f"    note: {len(glitches)} word(s) were corrupted in transit during "
              f"the bulk read ({shown}{more}) but read back correctly - the chip "
              "contents are good")
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


def main():
    ap = argparse.ArgumentParser(description="PIC16F72 programmer host (Arduino ICSP engine)")
    ap.add_argument('--port', required=True, help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument('--hex', help="Intel HEX file: full erase/blank/program/verify run")
    ap.add_argument('--erase', action='store_true', help="chip erase + blank check")
    ap.add_argument('--id', action='store_true', help="read device ID and exit")
    ap.add_argument('--dump', metavar='FILE', help="dump full memory to a text file")
    args = ap.parse_args()

    if not (args.hex or args.erase or args.id or args.dump):
        ap.error("nothing to do: give --hex, --erase, --id, or --dump")

    p = Programmer(args.port)
    try:
        if args.id:
            do_id(p)
        if args.dump:
            do_dump(p, args.dump)
        if args.erase:
            do_erase(p)
        if args.hex:
            do_full(p, args.hex)
    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        p.close()


if __name__ == '__main__':
    main()
