#!/usr/bin/env python3

import subprocess
from pathlib import Path
from vunit import VUnit

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "source"
COM = SOURCE / "fpga_communication"
FIXED = SOURCE / "hVHDL_fixed_point"

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

# hVHDL_fixed_point modules under test
lib.add_source_files(FIXED / "fixed_dsp/fixed_dsp.vhd")
lib.add_source_files(FIXED / "fixed_dsp/arch_rtl_fixed_dsp.vhd")

MEMORY = FIXED / "submodules/hVHDL_memory_library/vhdl2008"
lib.add_source_files(MEMORY / "dp_ram_w_configurable_recrods.vhd")
# the protected type simulation model, the builds use arch_rtl
lib.add_source_files(MEMORY / "arch_sim_dp_ram_w_configurable_records.vhd")
lib.add_source_files(FIXED / "lut_interpolation/lut_sine_pkg.vhd")
lib.add_source_files(FIXED / "sine_calculator/sine_calculator.vhd")
lib.add_source_files(FIXED / "lut_interpolation/lut_reciprocal_pkg.vhd")
lib.add_source_files(FIXED / "reciprocal_calculator/reciprocal_calculator.vhd")
lib.add_source_files(FIXED / "lut_interpolation/lut_sqrt_pkg.vhd")
lib.add_source_files(FIXED / "sqrt_calculator/sqrt_calculator.vhd")
lib.add_source_files(FIXED / "fixed_point_scaling/fixed_point_scaling_pkg.vhd")
lib.add_source_files(FIXED / "lut_divider/lut_divider.vhd")
lib.add_source_files(FIXED / "full_range_sqrt/full_range_sqrt.vhd")

lib.add_source_files(SOURCE / "git_hash_pkg.vhd")
lib.add_source_files(SOURCE / "lut_sweep.vhd")
lib.add_source_files(SOURCE / "divider_sweep.vhd")
lib.add_source_files(SOURCE / "sqrt_sweep.vhd")
lib.add_source_files(SOURCE / "uart_test_core.vhd")

lib.add_source_files(ROOT / "testbench/uart_test_core_tb.vhd")
core_tb = lib.test_bench("uart_test_core_tb")
core_tb.add_config(name="dsp", generics=dict(pre_add_register=False))
core_tb.add_config(name="dsp_pre_add_register", generics=dict(pre_add_register=True))

VU.main()
