python pic72_host.py --port COM5 --id              # smoke test: expect "PIC16F72 detected"
python pic72_host.py --port COM5 --hex firmware.hex # full erase/blank/program/verify
python pic72_host.py --port COM5 --dump dump.txt    # read the whole chip back