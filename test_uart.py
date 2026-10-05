#!/usr/bin/env python3
"""
test_uart.py - hardware test of the uart_test_core register interface

    python3 test_uart.py --board au         # Alchitry Au+,  5.0 Mbaud
    python3 test_uart.py --board axc3000    # Arrow AXC3000, 4.8 Mbaud
    python3 test_uart.py --board ti60evm    # Ti60F225 EVM,  4.8 Mbaud
    python3 test_uart.py --board au --port /dev/ttyUSB2 --rounds 2000

The serial port is found from the board's USB VID:PID and interface
number unless --port is given. Needs pyserial.

fpga_communication protocol, 16 bit address, 32 bit data, big endian :
    read   : 0x02 addr[2]                -> 7 byte frame, data in the last 4
    write  : 0x04 addr[2] data[4]
    stream : 0x05 addr[2] count[3]       -> count * data[4], one address repeated

Register map (source/uart_test_core.vhd) :
    1      id 0x0000ACDC                 RO
    2      git hash                      RO
    3      loopback                      RW
    4      read counter, ++ per read     RO
    5      board id                      RO
    6      core clock frequency, Hz      RO
    7      free running clock counter    RO
    16..31 register bank                 RW

Exit status 0 = all passed.
"""
import argparse
import random
import sys
import time
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("this script needs pyserial:  pip install pyserial")

# board id, baud, usb vid, pid, uart interface number
BOARDS = {
    "au":      dict(board_id=1, baud=5_000_000, vid=0x0403, pid=0x6010, interface=1),
    "axc3000": dict(board_id=2, baud=4_800_000, vid=0x09FB, pid=0x6022, interface=1),
    "ti60evm": dict(board_id=3, baud=4_800_000, vid=0x0403, pid=0x6011, interface=2),
}

ID_VALUE = 0x0000ACDC
BANK = range(16, 32)


class FpgaUart:
    def __init__(self, port, baud):
        self.uart = serial.Serial(port, baudrate=baud, timeout=0.5)
        self.uart.reset_input_buffer()

    def close(self):
        self.uart.close()

    def _read_exact(self, n):
        data = self.uart.read(n)
        if len(data) != n:
            raise TimeoutError(f"expected {n} bytes, got {len(data)}: {data.hex()}")
        return data

    def read(self, address):
        self.uart.write(bytes([0x02]) + address.to_bytes(2, "big"))
        return int.from_bytes(self._read_exact(7)[3:], "big")

    def write(self, address, data):
        self.uart.write(bytes([0x04]) + address.to_bytes(2, "big") + data.to_bytes(4, "big"))

    def stream(self, address, count):
        self.uart.write(bytes([0x05]) + address.to_bytes(2, "big") + count.to_bytes(3, "big"))
        data = self._read_exact(count * 4)
        return [int.from_bytes(data[i:i + 4], "big") for i in range(0, len(data), 4)]


def find_port(board):
    for p in serial.tools.list_ports.comports():
        if p.vid == board["vid"] and p.pid == board["pid"] and p.location \
                and p.location.endswith(f".{board['interface']}"):
            return p.device
    return None


def set_low_latency(port):
    """ftdi_sio defaults to a 16 ms latency timer, 1 ms makes the
    one-register-at-a-time reads ~10x faster ; needs write access to sysfs"""
    timer = Path(f"/sys/bus/usb-serial/devices/{Path(port).name}/latency_timer")
    try:
        if timer.exists() and timer.read_text().strip() != "1":
            timer.write_text("1")
    except OSError:
        pass


class Results:
    def __init__(self):
        self.failed = 0

    def check(self, name, ok, detail=""):
        print(f"  {'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
        if not ok:
            self.failed += 1


def run(uart, board, rounds, r):
    print("identity")
    r.check("id register = 0x0000ACDC", uart.read(1) == ID_VALUE, f"read 0x{uart.read(1):08X}")
    board_id = uart.read(5)
    r.check(f"board id = {board['board_id']}", board_id == board["board_id"], f"read {board_id}")
    clock_hz = uart.read(6)
    r.check("clock frequency register = 120 MHz", clock_hz == 120_000_000, f"read {clock_hz}")
    print(f"        git hash 0x{uart.read(2):08x}")

    print("loopback register")
    patterns = [0x00000000, 0xFFFFFFFF, 0xAAAAAAAA, 0x55555555, 0x80000001, 0x12345678]
    patterns += [1 << n for n in range(32)]
    bad = [p for p in patterns if (uart.write(3, p), uart.read(3))[1] != p]
    r.check(f"{len(patterns)} fixed patterns incl. walking one", not bad,
            ", ".join(f"0x{p:08X}" for p in bad[:4]))

    print("read counter")
    first = uart.read(4)
    counts = [uart.read(4) for _ in range(20)]
    expected = [(first + 1 + n) & 0xFFFFFFFF for n in range(20)]
    r.check("increments by one per read", counts == expected, f"{first} -> {counts[-1]}")

    print("core clock")
    t0, c0 = time.monotonic(), uart.read(7)
    time.sleep(2.0)
    t1, c1 = time.monotonic(), uart.read(7)
    measured_hz = ((c1 - c0) & 0xFFFFFFFF) / (t1 - t0)
    r.check("free running counter at 120 MHz +-1 %", abs(measured_hz / 120e6 - 1) < 0.01,
            f"measured {measured_hz / 1e6:.2f} MHz")

    print("register bank 16..31")
    bank_errors = 0
    for _ in range(max(1, rounds // 16)):
        values = {a: r.random.getrandbits(32) for a in BANK}
        for a, v in values.items():
            uart.write(a, v)
        bank_errors += sum(uart.read(a) != v for a, v in values.items())
    r.check(f"{max(1, rounds // 16)} rounds of 16 random words", bank_errors == 0,
            f"{bank_errors} wrong")
    r.check("loopback register untouched by bank writes", uart.read(3) == patterns[-1])

    print("stream")
    uart.write(16, 0xC0FFEE00)
    streamed = uart.stream(16, 256)
    r.check("256 word stream of one register", streamed == [0xC0FFEE00] * 256,
            f"{sum(v != 0xC0FFEE00 for v in streamed)} wrong")
    streamed = uart.stream(7, 64)
    increasing = all(((b - a) & 0xFFFFFFFF) < 1 << 31 and b != a for a, b in zip(streamed, streamed[1:]))
    r.check("clock counter stream is increasing", increasing)

    print(f"random write/read, {rounds} words")
    started = time.monotonic()
    errors = 0
    for _ in range(rounds):
        v = r.random.getrandbits(32)
        uart.write(3, v)
        errors += uart.read(3) != v
    elapsed = time.monotonic() - started
    r.check(f"{rounds} random words through the loopback register", errors == 0,
            f"{errors} wrong, {rounds / elapsed:.0f} round trips/s")

    uart.uart.reset_input_buffer()
    time.sleep(0.05)
    r.check("no stray bytes from the fpga", uart.uart.in_waiting == 0, f"{uart.uart.in_waiting} bytes")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--board", choices=BOARDS, required=True)
    parser.add_argument("--port", help="serial port, found from the usb ids if not given")
    parser.add_argument("--baud", type=float, help="override the board's baud rate")
    parser.add_argument("--rounds", type=int, default=500, help="random write/read round trips")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    board = BOARDS[args.board]
    port = args.port or find_port(board)
    if port is None:
        sys.exit(f"no serial port for {args.board} (usb {board['vid']:04x}:{board['pid']:04x} "
                 f"interface {board['interface']}), is it attached to wsl with usbipd?")
    baud = int(args.baud or board["baud"])
    set_low_latency(port)
    print(f"{args.board} on {port} at {baud / 1e6:g} Mbaud")

    r = Results()
    r.random = random.Random(args.seed)
    uart = FpgaUart(port, baud)
    try:
        run(uart, board, args.rounds, r)
    except TimeoutError as e:
        r.check("fpga answered", False, str(e))
    finally:
        uart.close()

    print("ALL PASSED" if r.failed == 0 else f"{r.failed} FAILED")
    return 0 if r.failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
