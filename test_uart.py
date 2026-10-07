#!/usr/bin/env python3
"""
test_uart.py - hardware test of the hw_test_core register interface

    python3 test_uart.py --board au         # Alchitry Au+,  5.0 Mbaud
    python3 test_uart.py --board axc3000    # Arrow AXC3000, 4.8 Mbaud
    python3 test_uart.py --board ti60evm    # Ti60F225 EVM,  4.8 Mbaud
    python3 test_uart.py --board trion      # Trion T120F324, spi at 10 MHz
    python3 test_uart.py --board au --port /dev/ttyUSB2 --rounds 2000

The serial port is found from the board's USB VID:PID and interface
number unless --port is given. Needs pyserial.

fpga_communication protocol, 16 bit address, 32 bit data, big endian :
    read   : 0x02 addr[2]                -> 7 byte frame, data in the last 4
    write  : 0x04 addr[2] data[4]
    stream : 0x05 addr[2] count[3]       -> count * data[4], one address repeated

Register map (source/hw_test_core.vhd) :
    1      id 0x0000ACDC                 RO
    2      git hash                      RO
    3      loopback                      RW
    4      read counter, ++ per read     RO
    5      board id                      RO
    6      core clock frequency, Hz      RO
    7      free running clock counter    RO
    16..31 register bank                 RW

    fixed_dsp(rtl), a/d/b n bits, c and result 2n bits (n = register 44)
    32 a   33 d   34 b   35 c low   36 c high                     RW
    37 control: 0 pre_subtract 1 post_subtract 2 invert 3 accumulate
    38 write N -> N back to back requests                        WO
    39 result low   40 result high                               RO
    41 latency, request at the dsp input to ready, clock edges   RO
    42 ready pulses of the last command                          RO
    43 write -> accumulator reset                                WO
    44 dsp word length n                                         RO
    45 1 when the pre-adder is registered (latency 3)             RO
    46 1 when the lookup table rams have their output register     RO
    47 1 when the dsp requests are registered                      RO
    125 1 when every fixed_dsp registers its product (one clock more) RO

    lut calculators through lut_sweep, 16 bit input, 16 bit result
    48.. sine_calculator, angle (fraction of a turn) -> signed sine
    64.. reciprocal_calculator, x_frac (x = 0.5 + x_frac/2**17) -> 1/x
    80.. sqrt_calculator, x_frac (x = 0.5 + x_frac/2**17) -> sqrt(x)

    lut_divider, 32 bit, quotient = numerator / denominator * 2**16
    96 numerator   97 denominator (also the lfsr seeds)            RW
    98 write -> one division   99 quotient   100 division by zero    WO/RO/RO
    101 latency    102 write N -> sweep N divisions (0 = 65536)      RO/WO
    103 sweep mode : bit 0 lfsr operands, bit 1 gaps                 RW
    104 s1   105 s2   106 ready pulses   107 division by zero count  RO
    108 table index width  109 table word length  110 table radix  111 x_frac width  RO

    full_range_sqrt, 32 bit, root = sqrt(radicand * 2**-16) * 2**16
    112 radicand (also the sweep start / lfsr seed)                RW
    113 write -> one root   114 root   115 latency                  WO/RO/RO
    116 write N -> sweep N roots (0 = 65536)                         WO
    117 sweep mode : bit 0 lfsr radicands, bit 1 gaps               RW
    118 s1   119 s2   120 ready pulses                              RO
    121 table index width  122 table word length  123 table radix  124 x_frac width  RO

    microprogram_core with execution_unit(fixed_mult_add), 32 bit data
    128 program start address   129 write -> run the program        RW/WO
    130 busy   131 ready pulses   132 clock edges from run to ready    RO
    133 radix                                                        RO
    134 bit 0 : boost converter model (program 128) in the background RW
    135 clock edges between background runs   136 background runs   RW/RO
    137 data word width   138 instruction width   139 data ram words  RO
    140 result latency L : the programs are scheduled for it      RO
    141 jump delay slots S, 2 without the program ram's register    RO
    142 the math unit's result latency, 0 without one               RO
        run times 0 : 12 + S + L, 32 : 4 + S + 100 L,
        128 : 3 + S + 3 L, 192 : 4 + S + L
    384..511 the data ram's bits above 31 (none at 32 bits)          RW

    the same with 36 bit data and instructions (8 bit address fields,
    a 256 word data ram) at radix 24 and a math unit (division, program
    224 : 112 <- 110 / 111, 113 <- 112 * 114 + 115, 116 <- 113 / 111) :
    144..158 registers   512..767 data ram bits 31..0   768..1023 bits 35..32
    256..383 data ram : write -> the ram, read <- a copy of it       RW
    base +0 input   +1 write -> one request   +2 result               RW/WO/RO
         +3 latency, request at the input to ready, clock edges     RO
         +4 write N -> sweep N inputs from +5, one per clock (0 = 65536) WO
         +5 sweep start   +6 sum of results s1   +7 sum of s1 s2    RW/RO/RO
         +8 ready pulses of the last command   +9 sweep mode, 1 = gaps RO/RW

Exit status 0 = all passed.
"""
import argparse
import math
import random
import sys
import time
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("this script needs pyserial:  pip install pyserial")

# board id, core clock, baud, usb vid, pid, uart interface number, and
# strings the usb product description must (product) or must not (not_product)
# contain : the Alchitry's FT2232 has the same ids as Efinix's Trion boards
BOARDS = {
    "au":      dict(board_id=1, clock_hz=120_000_000, baud=5_000_000, vid=0x0403, pid=0x6010, interface=1,
                    not_product=["Trion", "Titanium"]),
    # by its blaster's serial number : other USB Blaster IIIs (an Agilex 5
    # board) have the same ids
    "axc3000": dict(board_id=2, clock_hz=120_000_000, baud=4_800_000, vid=0x09FB, pid=0x6022, interface=1,
                    serial="TEA45543"),
    "ti60evm": dict(board_id=3, clock_hz=120_000_000, baud=4_800_000, vid=0x0403, pid=0x6011, interface=2,
                    product="Ti60F225"),
    # no uart on the trion board : spi from its FT2232H channel A, chosen by
    # the ftdi serial number
    "trion":   dict(board_id=4, clock_hz=60_000_000, spi_url="ftdi://ftdi:2232:FT56NF97/1", spi_hz=10e6),
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


class FpgaSpi:
    """the same register access over spi (efinix_spi_communication's
    fpga_spi_communications) : mode 0, a command followed by zero padding in
    one chip select low exchange, the response read from the bytes clocked
    back. after chip select falls the first byte back is 0xff, idle bytes
    are 0x00 and a response frame starts with its nonzero header."""

    PADDING = 24

    def __init__(self, url, frequency):
        from pyftdi.spi import SpiController
        self.controller = SpiController()
        self.controller.configure(url)
        self.port = self.controller.get_port(cs=0, freq=frequency, mode=0)
        self.latency = None

    def close(self):
        self.controller.terminate()

    def _exchange(self, data):
        return self.port.exchange(bytes(data), duplex=True)

    def write(self, address, data):
        self._exchange(bytes([0x04]) + address.to_bytes(2, "big") + data.to_bytes(4, "big"))

    def _read_frame(self, address):
        command = bytes([0x02]) + address.to_bytes(2, "big")
        rx = self._exchange(command + bytes(self.PADDING))
        start = next((i for i in range(1, len(rx)) if rx[i] != 0), None)
        if start is None or start + 7 > len(rx):
            raise TimeoutError(f"no spi response to the read of register {address}: {bytes(rx).hex()}")
        return rx, start - len(command)

    def read(self, address):
        rx, latency = self._read_frame(address)
        start = latency + 3
        return int.from_bytes(rx[start + 3:start + 7], "big")

    def stream(self, address, count):
        """count words of one address ; the words follow the command after
        the same number of bytes as a read response's header"""
        if self.latency is None:
            _, self.latency = self._read_frame(1)
        command = bytes([0x05]) + address.to_bytes(2, "big") + count.to_bytes(3, "big")
        rx = self._exchange(command + bytes(self.latency + 4 * count + 4))
        first = len(command) + self.latency
        return [int.from_bytes(rx[first + 4 * k:first + 4 * k + 4], "big") for k in range(count)]


def find_port(board):
    """the board's uart port, None when there is none, sys.exit when several
    attached boards match"""
    candidates = []
    for p in serial.tools.list_ports.comports():
        description = f"{p.description} {p.product or ''}"
        if p.vid == board["vid"] and p.pid == board["pid"] and p.location \
                and p.location.endswith(f".{board['interface']}") \
                and board.get("product", "") in description \
                and board.get("serial", p.serial_number) == p.serial_number \
                and not any(x in description for x in board.get("not_product", [])):
            candidates.append(p)
    if len(candidates) > 1:
        sys.exit("several boards match, choose one with --port : "
                 + ", ".join(f"{p.device} ({p.description}, serial {p.serial_number})" for p in candidates))
    return candidates[0].device if candidates else None


def set_low_latency(port):
    """ftdi_sio defaults to a 16 ms latency timer, 1 ms makes the
    one-register-at-a-time reads ~10x faster ; needs write access to sysfs"""
    timer = Path(f"/sys/bus/usb-serial/devices/{Path(port).name}/latency_timer")
    try:
        if timer.exists() and timer.read_text().strip() != "1":
            timer.write_text("1")
    except OSError:
        pass


def wrap(value, bits):
    """two's complement wrap to a signed bits wide value"""
    value &= (1 << bits) - 1
    return value - (1 << bits) if value >> (bits - 1) else value


PRE_SUBTRACT, POST_SUBTRACT, INVERT, ACCUMULATE = 1, 2, 4, 8


def fixed_dsp_model(a, d, b, c, control, n, requests=1):
    """bit exact model of hVHDL_fixed_point fixed_dsp(rtl) : the result
    register P starts from 0 (the core drives zeros while idle) and takes
    requests back to back"""
    pre = wrap(a - d if control & PRE_SUBTRACT else a + d, n)
    mult = pre * b
    p = 0
    for _ in range(requests):
        if control & ACCUMULATE:
            p = p - mult if control & POST_SUBTRACT else p + mult
        else:
            p = mult - c if control & POST_SUBTRACT else mult + c
            if control & INVERT:
                p = -p
        p = wrap(p, 2 * n)
    return p


def vhdl_round(x):
    """math_real round : halfway cases away from zero"""
    return int(math.floor(abs(x) + 0.5)) * (1 if x >= 0 else -1)


# lut_sine_pkg : quarter wave tables of 256 entries, 16 bit
SINE_ENTRIES = 256
SINE_SCALE = 2.0**15 - 1.0
SINE_POINT = [vhdl_round(math.sin(math.pi / 2.0 * i / SINE_ENTRIES) * SINE_SCALE) for i in range(SINE_ENTRIES)]
SINE_SLOPE = [vhdl_round((math.sin(math.pi / 2.0 * (i + 1) / SINE_ENTRIES)
                          - math.sin(math.pi / 2.0 * i / SINE_ENTRIES)) * SINE_SCALE) for i in range(SINE_ENTRIES)]


def sine_model(angle):
    """bit exact lut_sine_pkg.get_sine_from_quarter_wave_lut"""
    phase = angle & 0x3FFF
    if angle & 0x4000:
        phase = ~phase & 0x3FFF
    index, fraction = phase >> 6, phase & 0x3F
    value = wrap(SINE_POINT[index] + ((SINE_SLOPE[index] * fraction) >> 6), 16)
    return wrap(-value, 16) if angle & 0x8000 else value


# lut_reciprocal_pkg : 256 entries of 1/x over 0.5 <= x < 1, radix 14
RECIP_ENTRIES = 256
RECIP_SCALE = 2.0**14 - 1.0
recip_at = lambda i: 1.0 / (0.5 * (1.0 + i / RECIP_ENTRIES))
RECIP_POINT = [vhdl_round(recip_at(i) * RECIP_SCALE) for i in range(RECIP_ENTRIES)]
RECIP_SLOPE = [vhdl_round((recip_at(i + 1) - recip_at(i)) * RECIP_SCALE) for i in range(RECIP_ENTRIES)]


def reciprocal_model(x_frac):
    """bit exact lut_reciprocal_pkg.get_reciprocal_from_lut"""
    index, fraction = x_frac >> 8, x_frac & 0xFF
    return wrap(RECIP_POINT[index] + wrap((RECIP_SLOPE[index] * fraction) >> 8, 16), 16) & 0xFFFF


# lut_sqrt_pkg : 256 entries of sqrt(x) over 0.5 <= x < 1, radix 15
SQRT_ENTRIES = 256
SQRT_SCALE = 2.0**15 - 1.0
sqrt_at = lambda i: math.sqrt(0.5 * (1.0 + i / SQRT_ENTRIES))
SQRT_POINT = [vhdl_round(sqrt_at(i) * SQRT_SCALE) for i in range(SQRT_ENTRIES)]
SQRT_SLOPE = [vhdl_round((sqrt_at(i + 1) - sqrt_at(i)) * SQRT_SCALE) for i in range(SQRT_ENTRIES)]


def sqrt_model(x_frac):
    """bit exact lut_sqrt_pkg.get_sqrt_from_lut"""
    index, fraction = x_frac >> 8, x_frac & 0xFF
    return wrap(SQRT_POINT[index] + wrap((SQRT_SLOPE[index] * fraction) >> 8, 16), 16) & 0xFFFF


_reciprocal_tables = {}


def reciprocal_tables(index_width, word_length, radix):
    """lut_reciprocal_pkg.make_reciprocal_point_lut / _slope_lut"""
    key = (index_width, word_length, radix)
    if key not in _reciprocal_tables:
        entries, scale = 2**index_width, 2.0**radix - 1.0
        at = lambda i: 1.0 / (0.5 * (1.0 + i / entries))
        _reciprocal_tables[key] = ([wrap(vhdl_round(at(i) * scale), word_length) for i in range(entries)],
                                   [wrap(vhdl_round((at(i + 1) - at(i)) * scale), word_length) for i in range(entries)])
    return _reciprocal_tables[key]


def reciprocal_table_model(x_frac, x_frac_width, index_width, word_length, radix):
    """bit exact lut_reciprocal_pkg.get_reciprocal_from_lut(x_frac, point_lut, slope_lut)"""
    points, slopes = reciprocal_tables(index_width, word_length, radix)
    fraction_width = x_frac_width - index_width
    index, fraction = x_frac >> fraction_width, x_frac & ((1 << fraction_width) - 1)
    return wrap(points[index] + wrap((slopes[index] * fraction) >> fraction_width, word_length), word_length) \
        & ((1 << word_length) - 1)


DEFAULT_TABLE = (8, 16, 14, 16)  # index width, word length, radix, x_frac width


def lut_divide_model(numerator, denominator, radix, w=32, table=DEFAULT_TABLE):
    """bit exact lut_divider_pkg.lut_divide, table = (index width, word
    length, table radix, x_frac width)"""
    index_width, word_length, table_radix, x_frac_width = table
    if denominator == 0:
        return 0
    magnitude = abs(denominator)
    zeros = w - magnitude.bit_length()
    normalised = magnitude << zeros
    x_frac = (normalised >> (w - 1 - x_frac_width)) & ((1 << x_frac_width) - 1)
    product = numerator * reciprocal_table_model(x_frac, x_frac_width, index_width, word_length, table_radix)
    if denominator < 0:
        product = -product
    extra = max(0, radix - (table_radix + 1))
    return wrap((product << extra) >> (table_radix + w - radix + extra - zeros), w)


_sqrt_tables = {}


def sqrt_tables(index_width, word_length, radix):
    """lut_sqrt_pkg.make_sqrt_point_lut / _slope_lut"""
    key = (index_width, word_length, radix)
    if key not in _sqrt_tables:
        entries, scale = 2**index_width, 2.0**radix - 1.0
        at = lambda i: math.sqrt(0.5 * (1.0 + i / entries))
        _sqrt_tables[key] = ([wrap(vhdl_round(at(i) * scale), word_length) for i in range(entries)],
                             [wrap(vhdl_round((at(i + 1) - at(i)) * scale), word_length) for i in range(entries)])
    return _sqrt_tables[key]


def sqrt_table_model(x_frac, x_frac_width, index_width, word_length, radix):
    """bit exact lut_sqrt_pkg.get_sqrt_from_lut(x_frac, point_lut, slope_lut)"""
    points, slopes = sqrt_tables(index_width, word_length, radix)
    fraction_width = x_frac_width - index_width
    index, fraction = x_frac >> fraction_width, x_frac & ((1 << fraction_width) - 1)
    return wrap(points[index] + wrap((slopes[index] * fraction) >> fraction_width, word_length), word_length) \
        & ((1 << word_length) - 1)


DEFAULT_SQRT_TABLE = (8, 16, 15, 16)  # index width, word length, radix, x_frac width


def full_range_sqrt_model(radicand, radix, w=32, table=DEFAULT_SQRT_TABLE):
    """bit exact full_range_sqrt_pkg.get_full_range_sqrt, table = (index
    width, word length, table radix, x_frac width)"""
    index_width, word_length, table_radix, x_frac_width = table
    if radicand == 0:
        return 0
    zeros = w - radicand.bit_length()
    normalised = radicand << zeros
    x_frac = (normalised >> (w - 1 - x_frac_width)) & ((1 << x_frac_width) - 1)
    exponent = w - zeros + radix
    multiplier = 46341 if exponent % 2 else 32768
    product = sqrt_table_model(x_frac, x_frac_width, index_width, word_length, table_radix) * multiplier
    product_radix = table_radix + 15
    extra = max(0, (w + radix) // 2 - product_radix)
    return ((product << extra) >> (product_radix + extra - exponent // 2)) & ((1 << w) - 1)


def galois_step(x):
    return (x >> 1) ^ (0x80200003 if x & 1 else 0)


def root_sweep_model(mode, start, count, radix, table=DEFAULT_SQRT_TABLE):
    """sums of a sqrt_sweep sweep"""
    x = start & 0xFFFFFFFF
    s1 = s2 = 0
    for _ in range(count):
        if mode & 1:
            radicand = x >> (x & 0x1F)
            x = galois_step(x)
        else:
            radicand = x
            x = (x + 1) & 0xFFFFFFFF
        s1 = (s1 + full_range_sqrt_model(radicand, radix, table=table)) & 0xFFFFFFFF
        s2 = (s2 + s1) & 0xFFFFFFFF
    return s1, s2


def divider_sweep_model(mode, numerator, denominator, count, radix, table=DEFAULT_TABLE):
    """sums, division by zero count of a divider_sweep sweep"""
    n, d = numerator & 0xFFFFFFFF, denominator & 0xFFFFFFFF
    s1 = s2 = zeros = 0
    for _ in range(count):
        if mode & 1:
            divisor = wrap(d, 32) >> (n & 0x1F)
        else:
            divisor = wrap(d, 32)
            d = (d + 1) & 0xFFFFFFFF
        q = lut_divide_model(wrap(n, 32), divisor, radix, table=table)
        zeros += divisor == 0
        if mode & 1:
            n, d = galois_step(n), galois_step(d)
        s1 = (s1 + q) & 0xFFFFFFFF
        s2 = (s2 + s1) & 0xFFFFFFFF
    return (s1, s2), zeros


# base address, model, input edge cases
CALCULATORS = {
    "sine_calculator": (48, sine_model, [0, 1, 63, 64, 0x3FFF, 0x4000, 0x4001, 0x7FFF, 0x8000, 0xBFFF, 0xC000, 0xFFFF]),
    "reciprocal_calculator": (64, reciprocal_model, [0, 1, 0xFF, 0x100, 0x7FFF, 0x8000, 0xFEFF, 0xFF00, 0xFFFF]),
    "sqrt_calculator": (80, sqrt_model, [0, 1, 0xFF, 0x100, 0x7FFF, 0x8000, 0xFEFF, 0xFF00, 0xFFFF]),
}


def sweep_sums(model, start, count):
    s1 = s2 = 0
    for i in range(count):
        s1 = (s1 + model((start + i) & 0xFFFF)) & 0xFFFFFFFF
        s2 = (s2 + s1) & 0xFFFFFFFF
    return s1, s2


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
    r.check(f"clock frequency register = {board['clock_hz'] / 1e6:g} MHz", clock_hz == board["clock_hz"], f"read {clock_hz}")
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
    r.check(f"free running counter at {board['clock_hz'] / 1e6:g} MHz +-1 %", abs(measured_hz / board["clock_hz"] - 1) < 0.01,
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

    run_fixed_dsp(uart, rounds, r)
    for name, (base, model, edges) in CALCULATORS.items():
        run_lut_calculator(uart, name, base, model, edges, rounds, r)
    run_lut_divider(uart, rounds, r)
    run_full_range_sqrt(uart, rounds, r)
    run_microprogram_processor(uart, rounds, r)

    if not hasattr(uart, "uart"):
        return
    uart.uart.reset_input_buffer()
    time.sleep(0.05)
    r.check("no stray bytes from the fpga", uart.uart.in_waiting == 0, f"{uart.uart.in_waiting} bytes")


def dsp_request(uart, a, d, b, c, control, requests=1):
    """run one command on the fixed_dsp, returns (result, ready pulses)"""
    uart.write(32, a & 0xFFFFFFFF)
    uart.write(33, d & 0xFFFFFFFF)
    uart.write(34, b & 0xFFFFFFFF)
    uart.write(35, c & 0xFFFFFFFF)
    uart.write(36, (c >> 32) & 0xFFFFFFFF)
    uart.write(37, control)
    uart.write(38, requests)
    # a long burst can still be running when the first read arrives (1000
    # requests take 8.3 us, a read frame 6 us), wait for all the readies
    for _ in range(100):
        readies = uart.read(42)
        if readies >= requests:
            break
    result = wrap(uart.read(40) << 32 | uart.read(39), 64)
    return result, readies


def run_fixed_dsp(uart, rounds, r):
    print("fixed_dsp(rtl)")
    n = uart.read(44)
    r.check("dsp word length register", 2 <= n <= 32, f"n = {n}")

    def check_case(name, a, d, b, c, control, requests=1):
        expected = fixed_dsp_model(a, d, b, c, control, n, requests)
        result, readies = dsp_request(uart, a, d, b, c, control, requests)
        ok = result == expected and readies == requests
        if not ok or name:
            r.check(name or "random case", ok,
                    f"a={a} d={d} b={b} c={c} control={control:04b} x{requests} -> "
                    f"{result} expected {expected}, {readies} ready")
        return ok

    # the cases of hVHDL_fixed_point's fixed_dsp_tb, radix 16 operands and
    # radix 32 results
    one = 1 << 16
    fix = lambda x: int(round(x * one))
    check_case("add 2.5 * 1.5 = 3.75", fix(2.5), 0, fix(1.5), 0, 0)
    check_case("fmac (5 - 1) * 2 + 0.5 = 8.5", fix(5.0), fix(1.0), fix(2.0), fix(0.5) << 16, PRE_SUBTRACT)
    check_case("sub 3 * 2 - 1.25 = 4.75", fix(3.0), 0, fix(2.0), fix(1.25) << 16, POST_SUBTRACT)
    check_case("fmac invert -(3 * 2 + 1) = -7", fix(3.0), 0, fix(2.0), fix(1.0) << 16, INVERT)
    check_case("mac x8 of 1.5 * 1.25 = 15", fix(1.5), 0, fix(1.25), 0, ACCUMULATE, 8)
    check_case("mac subtract x8 of 1.5 * 1.25 = -15", fix(1.5), 0, fix(1.25), 0, ACCUMULATE | POST_SUBTRACT, 8)

    pre_add_register = uart.read(45)
    expected_latency = 2 + pre_add_register + uart.read(125)
    latency = uart.read(41)
    r.check(f"pipeline latency {expected_latency} clock edges"
            f"{' (pre-adder registered)' if pre_add_register else ''}", latency == expected_latency, f"read {latency}")

    lo, hi = -(1 << (n - 1)), (1 << (n - 1)) - 1
    check_case("min * min", lo, 0, lo, 0, 0)
    check_case("max * min - c max", hi, 0, lo, (1 << (2 * n - 1)) - 1, POST_SUBTRACT)
    check_case("pre adder wraps, max + 1", hi, 1, 3, 0, 0)
    check_case("pre subtract wraps, min - 1", lo, 1, 3, 0, PRE_SUBTRACT)
    check_case("result wraps, min * min + c max", lo, 0, lo, (1 << (2 * n - 1)) - 1, 0)
    check_case("invert of the most negative result", lo, 0, -lo - 1, -(1 << (2 * n - 1)) - lo * (-lo - 1), INVERT)
    check_case("1000 back to back accumulates of max * max", hi, 0, hi, 0, ACCUMULATE, 1000)

    uart.write(43, 1)
    reset_result = wrap(uart.read(40) << 32 | uart.read(39), 64)
    r.check("accumulator reset request", reset_result == 0 and uart.read(42) == 1, f"result {reset_result}")

    rnd = r.random
    cases = max(16, rounds // 4)
    failures = 0
    for _ in range(cases):
        width = rnd.choice([4, n // 2, n])
        operand = lambda: wrap(rnd.getrandbits(width), width) if width < n else wrap(rnd.getrandbits(n), n)
        control = rnd.getrandbits(4)
        requests = rnd.choice([1, 1, 1, 2, 3, rnd.randint(1, 300)])
        failures += not check_case(None, operand(), operand(), operand(), wrap(rnd.getrandbits(2 * n), 2 * n), control, requests)
    r.check(f"{cases} random requests, all flag combinations, bursts up to 300", failures == 0, f"{failures} wrong")


def lut_request(uart, base, value):
    uart.write(base + 0, value)
    uart.write(base + 1, 1)
    return wrap(uart.read(base + 2), 32)


def lut_sweep(uart, base, start, count, mode):
    """returns the sweep sums (s1, s2) and the ready pulses"""
    uart.write(base + 5, start)
    uart.write(base + 9, mode)
    uart.write(base + 4, count & 0xFFFF)
    for _ in range(100):
        readies = uart.read(base + 8)
        if readies >= count:
            break
    return (uart.read(base + 6), uart.read(base + 7)), readies


def first_wrong_input(uart, base, model, start, count, mode):
    """bisect with sweeps for the first input whose result differs"""
    while count > 1:
        half = count // 2
        sums, _ = lut_sweep(uart, base, start, half, mode)
        if sums == sweep_sums(model, start, half):
            start, count = (start + half) & 0xFFFF, count - half
        else:
            count = half
    return start


def run_lut_calculator(uart, name, base, model, edges, rounds, r):
    print(name)
    inputs = edges + [r.random.getrandbits(16) for _ in range(max(16, rounds // 8))]
    wrong = [(x, y, model(x)) for x in inputs if (y := lut_request(uart, base, x)) != model(x)]
    r.check(f"{len(inputs)} single inputs incl. edge cases", not wrong,
            ", ".join(f"{x} -> {y} expected {e}" for x, y, e in wrong[:4]))

    latency = uart.read(base + 3)
    expected_latency = 4 + uart.read(45) + uart.read(46) + uart.read(47) + uart.read(125)
    r.check(f"pipeline latency {expected_latency} clock edges", latency == expected_latency, f"read {latency}")

    sweeps = [(0, 65536, 0, "all 65536 inputs, back to back"), (0, 65536, 1, "all 65536 inputs, irregular gaps")]
    for _ in range(4):
        sweeps.append((r.random.getrandbits(16), r.random.randint(1, 5000), r.random.getrandbits(1), "random range"))
    for start, count, mode, sweep_name in sweeps:
        sums, readies = lut_sweep(uart, base, start, count, mode)
        ok = sums == sweep_sums(model, start, count) and readies == count
        detail = f"{count} inputs from {start}, {readies} ready"
        if not ok:
            x = first_wrong_input(uart, base, model, start, count, mode)
            detail += f", first wrong input {x}: {lut_request(uart, base, x)} expected {model(x)}"
        r.check(f"sweep {sweep_name}", ok, detail)


DIVIDER_BASE = 96
QUOTIENT_RADIX = 16


def divide(uart, numerator, denominator):
    uart.write(DIVIDER_BASE + 0, numerator & 0xFFFFFFFF)
    uart.write(DIVIDER_BASE + 1, denominator & 0xFFFFFFFF)
    uart.write(DIVIDER_BASE + 2, 1)
    return wrap(uart.read(DIVIDER_BASE + 3), 32), uart.read(DIVIDER_BASE + 4)


def divider_sweep(uart, mode, numerator, denominator, count):
    uart.write(DIVIDER_BASE + 0, numerator & 0xFFFFFFFF)
    uart.write(DIVIDER_BASE + 1, denominator & 0xFFFFFFFF)
    uart.write(DIVIDER_BASE + 7, mode)
    uart.write(DIVIDER_BASE + 6, count & 0xFFFF)
    for _ in range(100):
        readies = uart.read(DIVIDER_BASE + 10)
        if readies >= count:
            break
    return (uart.read(DIVIDER_BASE + 8), uart.read(DIVIDER_BASE + 9)), readies, uart.read(DIVIDER_BASE + 11)


def run_lut_divider(uart, rounds, r):
    table = tuple(uart.read(DIVIDER_BASE + k) for k in (12, 13, 14, 15))
    print(f"lut_divider, {2**table[0]} entries x {table[1]} bits at radix {table[2]}, {table[3]} bit x_frac")
    lo, hi = -2**31, 2**31 - 1
    pairs = [(1, 1), (1, -1), (-1, 1), (lo, 1), (lo, -1), (hi, lo), (lo, lo), (hi, hi), (1000, 3),
             (-1000, 7), (12345, 0), (0, 5), (5, hi), (123456789, -98765), (1, 2**30), (-77777, -3)]
    for _ in range(max(16, rounds // 8)):
        pairs.append((wrap(r.random.getrandbits(32), 32), wrap(r.random.getrandbits(32), 32) >> r.random.randint(0, 31)))
    wrong = []
    for n, d in pairs:
        q, dbz = divide(uart, n, d)
        if q != lut_divide_model(n, d, QUOTIENT_RADIX, table=table) or dbz != (d == 0):
            wrong.append(f"{n}/{d} -> {q} expected {lut_divide_model(n, d, QUOTIENT_RADIX, table=table)}")
    r.check(f"{len(pairs)} single divisions incl. edge cases", not wrong, ", ".join(wrong[:3]))

    # normalise 2 + reciprocal_calculator 4 + dsp + multiply 1 + dsp +
    # shift 2, both dsps one longer with the pre-adder register
    latency = uart.read(DIVIDER_BASE + 5)
    expected_latency = 10 + 2 * uart.read(45) + uart.read(46) + 2 * uart.read(47) + 2 * uart.read(125)
    r.check(f"pipeline latency {expected_latency} clock edges", latency == expected_latency, f"read {latency}")

    sweeps = [(0, 100000, -3000, 6000, "denominators -3000 .. 2999"),
              (0, -(2**31), 1, 65535, "most negative numerator, denominators 1 .. 65535"),
              (1, 0x1234567, 0x7654321, 65536, "65536 random pairs, back to back"),
              (3, 0x1357, 0x2468ACE, 65536, "65536 random pairs, irregular gaps")]
    for _ in range(2):
        sweeps.append((r.random.choice([1, 3]), r.random.getrandbits(31) | 1, r.random.getrandbits(31) | 1,
                       r.random.randint(1, 20000), "random seeds"))
    for mode, n, d, count, name in sweeps:
        sums, readies, zeros = divider_sweep(uart, mode, n, d, count)
        expected_sums, expected_zeros = divider_sweep_model(mode, n, d, count, QUOTIENT_RADIX, table)
        ok = sums == expected_sums and readies == count and zeros == expected_zeros
        r.check(f"sweep {name}", ok, f"{count} divisions, {readies} ready, {zeros} division by zero")

    # accuracy of the model itself against real division
    worst = 0.0
    for _ in range(20000):
        n = wrap(r.random.getrandbits(32), 32)
        d = wrap(r.random.getrandbits(32), 32) >> r.random.randint(0, 31)
        if d == 0:
            continue
        exact = n / d * 2**QUOTIENT_RADIX
        if 2**20 < abs(exact) < 2**31 - 2:
            worst = max(worst, abs(lut_divide_model(n, d, QUOTIENT_RADIX, table=table) - exact) / abs(exact))
    print(f"        lut_divide relative error up to {worst:.2e} for quotients above 2**20")


ROOT_BASE = 112
ROOT_RADIX = 16


def square_root(uart, radicand):
    uart.write(ROOT_BASE + 0, radicand)
    uart.write(ROOT_BASE + 1, 1)
    return uart.read(ROOT_BASE + 2)


def root_sweep(uart, mode, start, count):
    uart.write(ROOT_BASE + 0, start)
    uart.write(ROOT_BASE + 5, mode)
    uart.write(ROOT_BASE + 4, count & 0xFFFF)
    for _ in range(100):
        readies = uart.read(ROOT_BASE + 8)
        if readies >= count:
            break
    return (uart.read(ROOT_BASE + 6), uart.read(ROOT_BASE + 7)), readies


def run_full_range_sqrt(uart, rounds, r):
    table = tuple(uart.read(ROOT_BASE + k) for k in (9, 10, 11, 12))
    print(f"full_range_sqrt, {2**table[0]} entries x {table[1]} bits at radix {table[2]}, {table[3]} bit x_frac")
    radicands = [0, 1, 2, 3, 4, 1 << 16, 2 << 16, 4 << 16, 1 << 31, 0xFFFFFFFF, 0x12345678, 1 << 30]
    for _ in range(max(16, rounds // 8)):
        radicands.append(r.random.getrandbits(32) >> r.random.randint(0, 31))
    wrong = [f"{x:#x} -> {y} expected {full_range_sqrt_model(x, ROOT_RADIX, table=table)}"
             for x in radicands if (y := square_root(uart, x)) != full_range_sqrt_model(x, ROOT_RADIX, table=table)]
    r.check(f"{len(radicands)} single roots incl. edge cases", not wrong, ", ".join(wrong[:3]))

    # normalise 2 + sqrt_calculator 4 + dsp + multiply 1 + dsp + shift 2,
    # both dsps one longer with the pre-adder register
    latency = uart.read(ROOT_BASE + 3)
    expected_latency = 10 + 2 * uart.read(45) + uart.read(46) + 2 * uart.read(47) + 2 * uart.read(125)
    r.check(f"pipeline latency {expected_latency} clock edges", latency == expected_latency, f"read {latency}")

    sweeps = [(0, 0, 65536, "radicands 0 .. 65535"),
              (0, 0xFFFF0000, 65535, "radicands 0xffff0000 .. 0xfffffffe"),
              (1, 0x1234567, 65536, "65536 lfsr radicands, back to back"),
              (3, 0xBADCAFE, 65536, "65536 lfsr radicands, irregular gaps")]
    for _ in range(2):
        sweeps.append((r.random.choice([1, 3]), r.random.getrandbits(31) | 1, r.random.randint(1, 20000), "random seed"))
    for mode, start, count, name in sweeps:
        sums, readies = root_sweep(uart, mode, start, count)
        ok = sums == root_sweep_model(mode, start, count, ROOT_RADIX, table) and readies == count
        r.check(f"sweep {name}", ok, f"{count} roots, {readies} ready")

    worst = 0.0
    for _ in range(20000):
        x = r.random.getrandbits(32) >> r.random.randint(0, 31)
        exact = math.sqrt(x / 2**ROOT_RADIX) * 2**ROOT_RADIX
        if exact > 2**20:
            worst = max(worst, abs(full_range_sqrt_model(x, ROOT_RADIX, table=table) - exact) / exact)
    print(f"        full_range_sqrt relative error up to {worst:.2e} for roots above 2**20")


MPROCS = {"32 bit": dict(base=128, ram=256, ram_high=384),
          "36 bit": dict(base=144, ram=512, ram_high=768)}


def mult_add_model(a, b, c, radix, w=32):
    """bit exact model of fixed_mult_add on fixed_dsp : bits radix + w - 1
    downto radix of a * b + c * 2**radix, the operands signed w bit ; a
    sum, difference or -x in fixed_dsp's pre-adder wraps to w bits, c is
    added or subtracted at the product's width"""
    return wrap((a * b + (c << radix)) >> radix, w)


def mproc_ops_model(m, radix, w=32):
    """mproc_test.vhd's program 0 from the data ram words m[64..87], the
    results for addresses 1..8"""
    def ma(a, b, c):
        return mult_add_model(a, b, c, radix, w)
    return [ma(m[64], m[65], m[66]),
            ma(m[67], m[68], -m[69]),
            ma(wrap(-m[70], w), m[71], m[72]),
            ma(wrap(-m[73], w), m[74], -m[75]),
            ma(wrap(m[76] + m[77], w), m[78], 0),
            ma(wrap(m[79] - m[80], w), m[81], 0),
            ma(wrap(m[82] - m[83], w), m[84], m[83]),
            wrap(m[85] + m[86] + m[87], w)]


def mproc_filter_model(y, u, g, radix, w=32, rounds=100):
    """program 32 : rounds of lp_filter y <- (u - y) * g + y"""
    for _ in range(rounds):
        y = mult_add_model(wrap(u - y, w), g, y, radix, w)
    return y


BOOST = dict(vin=100, d=101, load=102, r=103, i_gain=104, u_gain=105, i=106, u=107)


def boost_steps(n, i, u, vin, d, load, r, i_gain, u_gain, radix, w=32):
    """n steps of mproc_test.vhd's boost converter model, program 128"""
    def ma(a, b, c):
        return mult_add_model(a, b, c, radix, w)
    for _ in range(n):
        vl = ma(wrap(-d, w), u, vin)
        ic = ma(d, i, -load)
        vl = ma(wrap(-r, w), i, vl)
        u = ma(ic, u_gain, u)
        i = ma(vl, i_gain, i)
    return i, u


class Mproc:
    """one mproc_test instance : its registers from base, its data ram's
    bits 31..0 from ram and the bits above from ram_high"""

    def __init__(self, uart, base, ram, ram_high):
        self.uart, self.base, self.ram, self.ram_high = uart, base, ram, ram_high
        self.radix = uart.read(base + 5)
        self.w = uart.read(base + 9)
        self.ram_size = uart.read(base + 11)
        # the result latency : the run times follow it, the programs are
        # laid out for it by the processor's schedule() and repeat()
        self.latency = uart.read(base + 12)
        # the jump delay slots, 2 without the program ram's output register
        self.slots = uart.read(base + 13)
        # the math unit's result latency, 0 without one
        self.math_latency = uart.read(base + 14)

    def write(self, address, value):
        if self.w > 32:
            self.uart.write(self.ram_high + address, (value >> 32) & 0xFFFFFFFF)
        self.uart.write(self.ram + address, value & 0xFFFFFFFF)

    def read(self, address):
        value = self.uart.read(self.ram + address)
        if self.w > 32:
            value |= self.uart.read(self.ram_high + address) << 32
        return wrap(value, self.w)

    def run(self, start):
        self.uart.write(self.base + 0, start)
        self.uart.write(self.base + 1, 1)
        for _ in range(100):
            if self.uart.read(self.base + 2) == 0:
                break
        return self.uart.read(self.base + 3), self.uart.read(self.base + 4)

    def write_boost(self, **values):
        for name, value in values.items():
            self.write(BOOST[name], value)

    def read_boost(self):
        return self.read(BOOST["i"]), self.read(BOOST["u"])


def run_microprogram_processor(uart, rounds, r):
    for name, addresses in MPROCS.items():
        run_mproc(Mproc(uart, **addresses), rounds, r)


def run_mproc(mp, rounds, r):
    w, radix = mp.w, mp.radix
    latency, slots = mp.latency, mp.slots
    print(f"microprogram processor, {w} bit data and {mp.uart.read(mp.base + 10)} bit instructions, "
          f"fixed_mult_add at radix {radix}, result latency {latency}, jump delay slots {slots}")
    mp.uart.write(mp.base + 6, 0)

    values = [wrap(r.random.getrandbits(w), w) for _ in range(mp.ram_size)]
    for k, v in enumerate(values):
        mp.write(k, v)
    wrong = sum(mp.read(k) != v for k, v in enumerate(values))
    r.check(f"{mp.ram_size} random {w} bit words through the data ram", wrong == 0, f"{wrong} wrong")

    edges = [0, 1, -1, 1 << radix, -(1 << radix), (1 << (w - 1)) - 1, -(1 << (w - 1))]
    wrong, runs = [], []
    for n in range(max(20, rounds // 25)):
        m = {k: r.random.choice(edges) if n % 4 == 0 else wrap(r.random.getrandbits(w), w)
             for k in range(64, 88)}
        for k, v in m.items():
            mp.write(k, v)
        runs.append(mp.run(0))
        got = [mp.read(k) for k in range(1, 9)]
        expected = mproc_ops_model(m, radix, w)
        wrong += [f"address {k + 1} {g} expected {e}" for k, (g, e) in enumerate(zip(got, expected)) if g != e]
    r.check(f"{len(runs)} runs of the 7 multiply-add commands and the accumulator", not wrong, ", ".join(wrong[:3]))
    r.check(f"program 0 ready once in {12 + slots + latency} clock edges", all(run == (1, 12 + slots + latency) for run in runs),
            f"ready pulses, clock edges {sorted(set(runs))}")

    wrong, runs = [], []
    for _ in range(10):
        y, u, g = (wrap(r.random.getrandbits(w) >> 6, w) for _ in range(3))
        for k, v in ((96, y), (97, u), (98, g)):
            mp.write(k, v)
        runs.append(mp.run(32))
        got, expected = mp.read(96), mproc_filter_model(y, u, g, radix, w)
        if got != expected:
            wrong.append(f"y {y} u {u} g {g} -> {got} expected {expected}")
    r.check("10 runs of a 100 round low pass filter loop", not wrong, ", ".join(wrong[:2]))
    r.check(f"program 32 ready once in {4 + slots + 100 * latency} clock edges", all(run == (1, 4 + slots + 100 * latency) for run in runs),
            f"ready pulses, clock edges {sorted(set(runs))}")

    if mp.ram_size > 128:
        wrong, runs = [], []
        for _ in range(10):
            m = {k: wrap(r.random.getrandbits(w), w) for k in range(200, 206)}
            for k, v in m.items():
                mp.write(k, v)
            runs.append(mp.run(192))
            expected = [mult_add_model(m[200], m[201], m[202], radix, w),
                        mult_add_model(m[203], m[204], -m[205], radix, w)]
            got = [mp.read(250), mp.read(251)]
            if got != expected:
                wrong.append(f"{got} expected {expected}")
        r.check(f"10 runs of program 192, operands and results above 127", not wrong, ", ".join(wrong[:2]))
        r.check(f"program 192 ready once in {4 + slots + latency} clock edges", all(run == (1, 4 + slots + latency) for run in runs),
                f"ready pulses, clock edges {sorted(set(runs))}")

    if mp.math_latency:
        run_math_unit(mp, r)

    run_boost_converter(mp, r)


# the math unit's lut_divider table : 512 x 18 bits at radix 16, 18 bit x_frac
MATH_TABLE = (9, 18, 16, 18)


def run_math_unit(mp, r):
    """program 224 : 112 <- 110 / 111, 113 <- 112 * 114 + 115,
    116 <- 113 / 111"""
    w, radix = mp.w, mp.radix
    print(f"        math unit, result latency {mp.math_latency}")

    def divide(n, d):
        return lut_divide_model(n, d, radix, w, MATH_TABLE)

    wrong, runs = [], []
    for _ in range(20):
        n = wrap(r.random.getrandbits(w), w)
        d = 0
        while d == 0:
            d = wrap(r.random.getrandbits(w), w) >> r.random.randint(0, w - 2)
        b, c = (wrap(r.random.getrandbits(w), w) for _ in range(2))
        for k, v in ((110, n), (111, d), (114, b), (115, c)):
            mp.write(k, v)
        runs.append(mp.run(224))
        q = divide(n, d)
        p = mult_add_model(q, b, c, radix, w)
        expected = [q, p, divide(p, d)]
        got = [mp.read(112), mp.read(113), mp.read(116)]
        if got != expected:
            wrong.append(f"{n} / {d} : {got} expected {expected}")
    r.check("20 runs of program 224, divisions and a multiply-add between them", not wrong, ", ".join(wrong[:2]))
    r.check("program 224 ready once", all(run[0] == 1 for run in runs), f"ready pulses {sorted(set(run[0] for run in runs))}")


def run_boost_converter(mp, r):
    uart, base, radix, w = mp.uart, mp.base, mp.radix, mp.w

    def fixed(x):
        return int(round(x * 2**radix))

    def real(x):
        return x / 2**radix

    uart.write(base + 6, 0)
    gains = dict(r=fixed(0.8), i_gain=fixed(0.7 / 3), u_gain=fixed(0.7 / 3))

    # single steps from the host, random operating points
    wrong, runs = [], []
    for _ in range(10):
        p = dict(vin=fixed(r.random.uniform(5, 30)), d=fixed(r.random.uniform(0.3, 0.9)),
                 load=fixed(r.random.uniform(0, 2)), **gains)
        i, u = fixed(r.random.uniform(-2, 2)), fixed(r.random.uniform(0, 40))
        mp.write_boost(i=i, u=u, **p)
        for _ in range(5):
            runs.append(mp.run(128))
            i, u = boost_steps(1, i, u, radix=radix, w=w, **p)
            got = mp.read_boost()
            if got != (i, u):
                wrong.append(f"{got} expected {(i, u)}")
    r.check(f"{len(runs)} boost converter steps run from the host", not wrong, ", ".join(wrong[:2]))
    r.check(f"program 128 ready once in {3 + mp.slots + 3 * mp.latency} clock edges",
            all(run == (1, 3 + mp.slots + 3 * mp.latency) for run in runs),
            f"ready pulses, clock edges {sorted(set(runs))}")

    # in the background, replayed for the number of runs it made
    p = dict(vin=fixed(10.0), d=fixed(0.5), load=fixed(0.25), **gains)
    i, u = fixed(1.0), fixed(5.0)
    mp.write_boost(i=i, u=u, **p)
    uart.write(base + 7, 2000)
    uart.write(base + 6, 1)
    time.sleep(0.05)
    uart.write(base + 6, 0)
    for _ in range(100):
        if uart.read(base + 2) == 0:
            break
    n = uart.read(base + 8)
    expected = boost_steps(n, i, u, radix=radix, w=w, **p)
    got = mp.read_boost()
    r.check(f"{n} background steps", n > 0 and got == expected,
            f"i {real(got[0]):.6f} A, u {real(got[1]):.6f} V")

    # change the load while it runs : it settles at i = load / d,
    # u = (vin - r i) / d
    uart.write(base + 6, 1)
    settled = []
    for load in (0.5, 1.5, 0.0):
        mp.write_boost(load=fixed(load))
        time.sleep(0.05)
        i, u = mp.read_boost()
        i_ss = load / 0.5
        u_ss = (10.0 - 0.8 * i_ss) / 0.5
        settled.append(abs(real(i) - i_ss) < 1e-3 and abs(real(u) - u_ss) < 1e-3)
        print(f"        load {load} A : i {real(i):.6f} A (steady state {i_ss:.4f}), u {real(u):.6f} V ({u_ss:.4f})")
    runs = uart.read(base + 8)
    uart.write(base + 6, 0)
    r.check("settles at the steady state after load steps written while it runs", all(settled), f"{runs} background steps")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--board", choices=BOARDS, required=True)
    parser.add_argument("--port", help="serial port, found from the usb ids if not given")
    parser.add_argument("--baud", type=float, help="override the board's baud rate")
    parser.add_argument("--rounds", type=int, default=500, help="random write/read round trips")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    board = BOARDS[args.board]
    if "spi_url" in board:
        return run_board(board, FpgaSpi(board["spi_url"], board["spi_hz"]),
                         f"{args.board} on {board['spi_url']} spi at {board['spi_hz'] / 1e6:g} MHz", args)
    port = args.port or find_port(board)
    if port is None:
        sys.exit(f"no serial port for {args.board} (usb {board['vid']:04x}:{board['pid']:04x} "
                 f"interface {board['interface']}), is it attached to wsl with usbipd?")
    baud = int(args.baud or board["baud"])
    set_low_latency(port)
    return run_board(board, FpgaUart(port, baud), f"{args.board} on {port} at {baud / 1e6:g} Mbaud", args)


def run_board(board, uart, description, args):
    print(description)
    r = Results()
    r.random = random.Random(args.seed)
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
