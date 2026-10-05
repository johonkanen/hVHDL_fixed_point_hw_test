#!/usr/bin/env python3

import subprocess
from pathlib import Path
from vunit import VUnit

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "source"
COM = SOURCE / "fpga_communication"

if not (SOURCE / "git_hash_pkg.vhd").exists():
    subprocess.run([ROOT / "write_git_hash.sh"], check=True)

VU = VUnit.from_argv(compile_builtins=True, vhdl_standard="2008")

lib = VU.add_library("lib")

# fpga_communication uart, 32 bit data / 16 bit address
lib.add_source_files(COM / "hVHDL_uart/uart_rx/uart_rx_pkg.vhd")
lib.add_source_files(COM / "hVHDL_uart/uart_tx/uart_tx_pkg.vhd")
lib.add_source_files(COM / "serial_protocol_generic_pkg.vhd")
lib.add_source_files(COM / "hVHDL_fpga_interconnect/fpga_interconnect_generic_pkg.vhd")
lib.add_source_files(COM / "communications.vhd")
lib.add_source_files(SOURCE / "fpga_interconnect_32_16_pkg.vhd")

lib.add_source_files(SOURCE / "git_hash_pkg.vhd")
lib.add_source_files(SOURCE / "uart_test_core.vhd")

lib.add_source_files(ROOT / "testbench/uart_test_core_tb.vhd")

VU.main()
