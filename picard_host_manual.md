# PIC16F72 Programmer — Command Manual (PicArd Revised Edition)

PC side: `picard_host.py` · Arduino side: `picard_revised/picard_revised.ino`
Reference host version: md5 `1ae5dc29ae5a70d0f38a99a0109703f9`

---

## 1. Basics

```
python picard_host.py --port COM11 <one action>
```

| Rule | Detail |
|---|---|
| `--port` | **Required**, always. e.g. `COM11`, `/dev/ttyUSB0`. |
| Baud | Fixed 115200 8N1 — no flag needed. |
| Startup | The Arduino auto-resets on port open; the script waits 2.2 s, then handshakes (`P72`). |
| Exit | Every run sends `X` before closing, which drops VPP, VDD and the boost. |
| One action per run | Flags can be combined (execution order: `--id`, `--dump`, `--erase`, `--verify`, `--hex`, `--boost`, `--boost-test`, `--set-target`, `--hold`, `--load-test`) but normally use one. `--hex` + `--verify` is rejected. |
| Requirements | `pip install pyserial`; chip in the ZIF, Arduino on the port, boost working. |

---

## 2. Command summary

| Command | Erases chip? | Typical time | Purpose |
|---|---|---|---|
| `--id` | no | ~4 s | Read device ID / check wiring |
| `--erase` | **yes** | 2–6 s | Erase + blank check |
| `--hex FILE` | **yes** | 25–55 s | Full erase → program → verify (self-repairing) |
| `--verify FILE` | no | 10–20 s | Read the chip and compare with a hex file |
| `--dump FILE` | no | ~20 s | Save the whole chip to a text file |
| `--boost` | no | interactive | Calibrate the 13 V VPP target |
| `--boost-test` | no | ~12 s | Run the boost with no load (VDD/VPP off) |
| `--set-target N` | no | ~3 s | Store a boost target in EEPROM |
| `--hold [SECS]` | no | SECS + 5 s | Hold VPP/VDD on the chip so you can meter it |
| `--load-test [N]` | **yes\*** | N × 14 ms | Worst-case programming-load test |

\* programs zeros over the chip's code; the next `--hex` erases anyway.

---

## 3. Detailed reference

### `--id`

```
python picard_host.py --port COM11 --id
```

Enters programming mode and reads the device-ID word at `0x2006` **three times** (majority vote).
Expects ID 5 → prints e.g. `PIC16F72 detected, silicon rev 2 (ID word 0x00a2)`.
Fails with a wiring/VPP hint if the ID is wrong. Use it as the first test after any rewiring.

### `--erase`

```
python picard_host.py --port COM11 --erase
```

Device check → bulk erase (`0x09`, self-timed 30 ms) → blank check of all 2048 words.
The blank check is glitch-tolerant: a word that reads used only in transit is re-read and
not reported as a failure. Prints `Chip erased, blank check passed` and the elapsed time.

### `--hex FILE`

```
python picard_host.py --port COM11 --hex bench_power_supply.hex
```

The full DS39588A flow:

1. **`[1]` Parse** the Intel HEX (validates checksums, word alignment, 14-bit range, memory map).
   Warns if no config word at `0x2007`.
2. **`[2]` Device ID** check.
3. **Erase + blank check.**
4. **`[3]` Program** every word present in the file; addresses not in the file stay erased (`0x3FFF`).
5. **`[4]` Program config/ID words** at `0x2000–0x2007`. A device-ID word at `0x2006` is skipped
   (factory-written, read-only).
6. **VPP span** printed: `A0 min-max (target N), duty peak d/560` plus the deepest dip in volts.
7. **Verify loop, up to 3 rounds:** full read → triage (transit glitch / real error / unstable) →
   reprogram **only** the words that really failed → read again. Notes tell you how many words
   needed a second pass and whether any address failed more than once.
8. **Config verify** with mask `0x005F` (bits 13–7 and 5 are unimplemented and read back as 1s).
9. Prints `[5] Verify passed` and exits programming mode.

What the notes mean:

- `every mismatch is an erased cell…` → cell never charged; repairable, and points at VPP/VDD margin.
- `…0s where the hex has 1s…` → data latched low (noise or bad command); **not** repairable by
  rewriting, needs another erase pass.
- `N word(s) were corrupted in transit…` → read-path noise, chip content is fine.
- `those words programmed on retry…` → transient margin problem, not a dead cell.

### `--verify FILE`  (read-only check against the hex)

```
python picard_host.py --port COM11 --verify bench_power_supply.hex
```

Reads all 2048 words + config **without erasing or programming**, majority-re-reads anything
suspicious, then prints either

```
MATCH - chip holds all 1041 program words and 1 config word(s) from bench_power_supply.hex
```

or a table:

```
MISMATCH
    1 program word(s) differ:
      0x0002  hex 30FF  chip 30FB  bits off 0004
    configuration word(s) differ:
      0x2007  hex 005A (masked)  chip 0042
```

Addresses **not** in the hex are compared against blank (`0x3FFF`).
This is the command to prove a programmed chip is correct.

### `--dump FILE`

```
python picard_host.py --port COM11 --dump chip.txt
```

Reads the chip **twice**, resolves differences by re-reading (majority), then reads all 8 config
words (3 reads each) and writes a text listing:

```
addr  word
0000  2805
...
2007  005A
```

2056 lines. Use when you want the raw contents rather than a pass/fail.

### `--boost` — calibrate VPP

```
python picard_host.py --port COM11 --boost
```

Enters programming mode and keeps VPP on MCLR, prints the raw A0 reading and the current target,
then asks you to type the voltage you measure with a meter (5–20 V accepted). Prints:

```
Measured 13.02V at ADC 883.
Suggested target for 13.0V: 882
Save it with:  python picard_host.py --port COM11 --set-target 882
```

It only *suggests*; nothing is stored until you run `--set-target`. The Arduino drops VPP after
120 s of no commands, so answer the prompt promptly.

### `--boost-test` — no-load boost test

```
python picard_host.py --port COM11 --boost-test
```

Runs the boost for 1.2 s with **VDD and VPP off** (no chip involvement), reports final ADC,
PWM duty, and the min/max ADC over the last 400 ms. Then keeps switching for 10 s so you can
meter the rail. Built-in diagnoses:

- big min/max swing → loop oscillating
- ADC near 0 with duty at ceiling → feedback node not reaching A0 (R2/R3, A0 wire, reversed zener)
- ADC pinned ≥1015 → zener clamp / divider swap
- duty pegged and still short → L1, D2, R10, Q4

### `--set-target N`

```
python picard_host.py --port COM11 --set-target 885
```

Sets the boost's ADC target (accepted range **300–1023**) and saves it in the Arduino's EEPROM
(survives power cycles). Rule of thumb: 13.0 V ≈ 885 with the calibrated board; always derive N
with `--boost` rather than guessing, because the ADC reference is the 5 V rail.

### `--hold [SECS]`

```
python picard_host.py --port COM11 --hold 30
```

Default 20 s. Enters programming mode via `V` and **leaves VPP/VDD on the chip**, printing once
a second:

```
    1s  A0  888  duty 455
```

and a summary at the end: `Over the window: A0 879-890, duty peak 560/560`.
Use it to meter **VDD at pin 20** and **VPP at MCLR pin 1** under a real static load.
Do *not* meter MCLR during a write — the meter aliases the ~75 Hz pulse train.

### `--load-test [N]`

```
python picard_host.py --port COM11 --load-test 300
```

Default 40. Enters programming mode and programs **N all-zero words at the real 13.8 ms cadence** —
worst-case pulse current, since every bit charges — then reports:

```
300 all-zero words programmed in 4149 ms (worst-case pulse current)
VPP during pulses: A0 879-890 (target 885), duty peak 560/560
  target 13.00V, low point 12.91V, dip 0.09V
```

Use N ≥ 300 for a steady-state number (40 words understates the dip). It leaves the chip holding zeros.

---

## 4. Common workflows

| Goal | Command |
|---|---|
| First power-up / after rewiring | `--id` |
| Program a chip | `--hex firmware.hex` |
| Prove a chip matches the hex (no rewrite) | `--verify firmware.hex` |
| Save the chip contents | `--dump chip.txt` |
| Set VPP to 13.00 V | `--boost`, note the suggestion, then `--set-target N` |
| Check rails under load | `--hold 30` with a meter on pins 1 and 20 |
| Stress-test VPP droop | `--load-test 300` |
| Test the boost alone | `--boost-test` |

---

## 5. Errors you may see

| Message | Meaning |
|---|---|
| `Unexpected handshake response… (is the picard_revised sketch loaded?)` | Wrong port, or the sketch isn't running/reset. |
| `Arduino rejected '…'` (`ER`) | Malformed command — usually a serial glitch; re-run. |
| `VPP regulation failed…` (`EV`) | The boost never reached target, so programming mode wasn't entered. Run `--boost-test`. |
| `Device ID … expected 5` | Wiring, VPP/VDD out of window, or chip not in the socket. |
| `Blank check failed at…` | Erase didn't complete; retry, or the chip is write-protected. |
| `all 2048 words read back 0x0000` | Code protection on, or programming failed. |
| `…reads back consistently wrong…` | Real write failure that survived the repair passes. |
| `…repeated reads … keep changing…` | Flaky connection — reseat PGC/PGD. |
| `Config verify failed…` | Config word didn't take. |

`Ctrl+C` aborts cleanly — VPP/VDD are still dropped on exit.

---

## 6. Hex file requirements

- Intel HEX (`:10…` records), checksums validated.
- Each instruction is a **14-bit word**, little-endian bytes, even byte count, even byte address.
- Code must lie in `0x0000–0x07FF`; config words in `0x2000–0x2007`
  (byte address `0x4000–0x400E` in the file).
- Words not present in the file are left erased. A device-ID word at `0x2006` is ignored.

---

## 7. Raw serial protocol (if you talk to the Arduino yourself)

115200 8N1, ASCII, commands terminated by `\n`. Case-sensitive.

| Command | Reply | Meaning |
|---|---|---|
| `?` | `P72` | Handshake |
| `E` | `OK` / `EV` | Enter programming mode (PC = 0x0000) |
| `X` | `OK` | Exit (VPP, VDD, boost off) |
| `I` | `Dxxxx` | Device-ID word at 0x2006 |
| `A` | `OK` | Bulk erase (30 ms, self-timed) |
| `C` | `OK` | Load configuration (PC = 0x2000) |
| `N` | `OK` | Increment address |
| `Pxxxx` | `OK` | Program 4-hex-digit word at PC, then increment |
| `R` | `Dxxxx` | Read word at PC (no increment) |
| `Gxxxx` | hex stream | Read `xxxx` words from PC (increments) |
| `S` | `Dxxxx yyyy` | Live A0 reading and PWM duty (0–800) |
| `B` | `Dxxxx yyyy mmmm nnnn` | Boost self-test (ADC, duty, min, max) |
| `T` | `Dxxxx yyyy zzzz` | VPP min, max, duty peak since programming mode began |
| `Qxxxx` | `Dxxxx yyyy zzzz mmmm` | Program `xxxx` zero words; min, max, duty peak, ms |
| `V` | `Dxxxx` | Enter programming mode and report A0 (VPP stays on) |
| `Vxxxx` | `OK` | Set boost target (300–1023) and save to EEPROM |
| `W` | `Dxxxx` | Report boost target |
| anything else | `ER` | Rejected |

`EV` = VPP regulation failed. If no command arrives for **120 s** while in programming mode, the
Arduino drops VPP/VDD/boost by itself.

Firmware constants: `TPROG_MS 3` (DS39588A maximum), `TERASE_MS 30`, `PWM_TOP 799` (20 kHz),
`DUTY_MAX 560`, `SETTLE_TOL 12`, `TARGET_DEFAULT 950`, 120 s idle timeout.
Host constants: `CONFIG_MASK 0x005F`, `REPAIR_PASSES 3`, `VPP_TARGET_VOLTS 13.0`.

---

## 8. Bench notes

- VPP must be **12.75–13.25 V** at the PIC's MCLR (pin 1) during the whole programming pulse;
  VDD **4.75–5.25 V** at pin 20. Both rails are checked with `--hold`.
- The ideal 13.0 V set point is ADC ≈ 885 on the calibrated board; the ADC references the 5 V rail,
  so recalibrate with `--boost` if the supply changes.
- A 3 ms pulse at ~17 mA needs local charge: a 470 µF/25 V in parallel with C1 keeps the rail dip
  around 0.1 V. A 100 nF ceramic at the socket's MCLR and VDD pins suppresses the µs-scale step the
  ADC cannot see.
- Read-back bursts of consecutive misread words are wiring/noise, not chip content: the host
  majority-re-reads them and `--verify` reports them separately from real mismatches.
