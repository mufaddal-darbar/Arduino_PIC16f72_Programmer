#!/usr/bin/env python3
"""Verify a pic72_host.py --dump output against an Intel HEX file.

Compares every program-memory word (0x0000-0x07FF) and every config word
the HEX file contains (0x2000-0x2007), then cross-checks the DS39588A
Table 4-1 checksum. Standalone: needs only Python 3, no pyserial.

Usage:
    python verify_dump.py dump.txt firmware.hex

Exit codes: 0 = match, 1 = mismatch, 2 = file/parse error.
"""

import argparse
import sys

BLANK = 0x3FFF
CODE_START, CODE_END = 0x0000, 0x07FF
CFG_START, CFG_END = 0x2000, 0x2007
DEVID_ADDR = 0x2006
# DS39588A Register 3-1: config-word bits 13-7 and 5 are unimplemented and
# read back as '1' (reserved locations behave the same way).
MASKED_CFG_ADDRS = range(0x2004, 0x2008)
CONFIG_MASK = 0x005F


def parse_hex(path):
    """Parse Intel HEX into ({code addr: word}, {config addr: word})."""
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
                raise ValueError(f"line {lineno}: odd byte count {nb} - not word-aligned")
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


def parse_dump(path):
    """Parse a dump file of 'addr  word' lines into {addr: word}."""
    words = {}
    with open(path) as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.strip()
            if not line:
                continue
            parts = line.split()
            if parts and parts[0].lower() == 'addr':      # header line
                continue
            if len(parts) != 2:
                raise ValueError(f"dump line {lineno}: expected 'addr word', got {line!r}")
            try:
                addr = int(parts[0], 16)
                word = int(parts[1], 16)
            except ValueError:
                raise ValueError(f"dump line {lineno}: malformed address or word: {line!r}")
            if not 0 <= word <= 0x3FFF:
                raise ValueError(f"dump line {lineno}: word {word:#06x} exceeds 14 bits")
            if addr in words and words[addr] != word:
                raise ValueError(f"dump line {lineno}: {addr:#06x} appears twice "
                                 "with different values")
            words[addr] = word
    return words


def checksum_code(words):
    """DS39588A Table 4-1, code protection off: SUM[0x0000:0x07FF] + CFWD&0x005F."""
    total = 0
    for a in range(CODE_START, CODE_END + 1):
        total += words.get(a, BLANK)
    total += words.get(0x2007, BLANK) & CONFIG_MASK
    return total & 0xFFFF


def main():
    ap = argparse.ArgumentParser(description="Verify a chip dump against an Intel HEX file.")
    ap.add_argument('dump', help="output of: pic72_host.py --dump FILE")
    ap.add_argument('hex', help="Intel HEX file the chip was programmed from")
    args = ap.parse_args()

    try:
        code, config = parse_hex(args.hex)
    except (OSError, ValueError) as e:
        print(f"error: {args.hex}: {e}", file=sys.stderr)
        return 2
    try:
        words = parse_dump(args.dump)
    except (OSError, ValueError) as e:
        print(f"error: {args.dump}: {e}", file=sys.stderr)
        return 2

    n_code_dump = sum(1 for a in words if CODE_START <= a <= CODE_END)
    print(f"hex:   {len(code)} program words, {len(config)} config/ID words")
    print(f"dump:  {n_code_dump} program words, "
          f"{sum(1 for a in words if CFG_START <= a <= CFG_END)} config words")
    print()

    problems = []

    mism, missing = [], []
    for a in range(CODE_START, CODE_END + 1):
        if a not in words:
            missing.append(a)
            continue
        expect = code.get(a, BLANK)
        if words[a] != expect:
            mism.append((a, expect, words[a]))
    if missing:
        shown = ', '.join(f"{a:04X}" for a in missing[:8])
        more = f" (+{len(missing) - 8} more)" if len(missing) > 8 else ''
        problems.append(f"{len(missing)} program address(es) missing from dump "
                        f"({shown}{more})")
    print(f"code    0x0000-0x07FF : {len(mism)} mismatch(es) "
          f"of {CODE_END - CODE_START + 1} words")
    for a, e, g in mism[:10]:
        print(f"    {a:04X}: hex {e:04X}  dump {g:04X}")
    if len(mism) > 10:
        print(f"    ... and {len(mism) - 10} more")
    if mism:
        problems.append(f"{len(mism)} program word(s) differ")

    for a in sorted(config):
        if a == DEVID_ADDR:
            print("note: hex contains a device-ID word at 0x2006 - not compared "
                  "(factory-programmed, read-only)")
            continue
        if a not in words:
            problems.append(f"config address {a:#06x} missing from dump")
            continue
        got, want = words[a], config[a]
        if a in MASKED_CFG_ADDRS:
            match = (got & CONFIG_MASK) == (want & CONFIG_MASK)
            label = f"config  {a:04X}        : hex {want:04X}  dump {got:04X} " \
                    f"(mask {CONFIG_MASK:04X}) - "
            if match:
                print(label + "match")
                if got != want:
                    print("    note: extra bits are unimplemented bits reading as 1s - OK")
            else:
                print(label + "MISMATCH")
                problems.append(f"config word at {a:#06x} differs")
        else:
            if got == want:
                print(f"ID loc  {a:04X}        : hex {want:04X}  dump {got:04X} - match")
            else:
                print(f"ID loc  {a:04X}        : hex {want:04X}  dump {got:04X} - MISMATCH")
                problems.append(f"ID location {a:#06x} differs")

    if DEVID_ADDR in words:
        w = words[DEVID_ADDR]
        print(f"device ID 0x2006       : {w:04X} "
              f"(device {(w >> 5) & 0x1FF}, revision {w & 0x1F})")

    chip_sum = checksum_code(words)
    hex_sum = checksum_code({**{a: code.get(a, BLANK) for a in range(CODE_START, CODE_END + 1)},
                             0x2007: config.get(0x2007, BLANK)})
    print(f"checksum (DS39588A 4-1): chip {chip_sum:04X}  hex {hex_sum:04X} - "
          f"{'match' if chip_sum == hex_sum else 'MISMATCH'}")
    if chip_sum != hex_sum:
        problems.append("datasheet checksum differs")

    print()
    if problems:
        print(f"RESULT: FAIL - {', '.join(problems)}")
        return 1
    print("RESULT: PASS - the chip contains exactly the hex file's contents")
    return 0


if __name__ == '__main__':
    sys.exit(main())
