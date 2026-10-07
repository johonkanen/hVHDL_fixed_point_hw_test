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

# spi link for boards without a uart (efinix_spi_communication)
SPI = SOURCE / "efinix_spi_communication/source"
lib.add_source_files(SPI / "spi_secondary.vhd")
lib.add_source_files(SPI / "spi_communications.vhd")

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

# hVHDL_microprogam_processor, with the fixed point and memory library
# above instead of its own submodules
MPROC = SOURCE / "hVHDL_microprogam_processor/rtl"
lib.add_source_files(MEMORY / "mpram_w_configurable_records.vhd")
lib.add_source_files(MPROC / "generic_microinstruction_pkg.vhd")
lib.add_source_files(MPROC / "microinstruction_pkg.vhd")
lib.add_source_files(MPROC / "microprogram_interface_pkg.vhd")
lib.add_source_files(MPROC / "execution_unit.vhd")
lib.add_source_files(MPROC / "arch_fixed_mult_add.vhd")
lib.add_source_files(MPROC / "microprogram_sequencer.vhd")
lib.add_source_files(MPROC / "microprogram_core.vhd")

lib.add_source_files(SOURCE / "git_hash_pkg.vhd")
lib.add_source_files(SOURCE / "mproc_test.vhd")
lib.add_source_files(SOURCE / "lut_sweep.vhd")
lib.add_source_files(SOURCE / "divider_sweep.vhd")
lib.add_source_files(SOURCE / "sqrt_sweep.vhd")
lib.add_source_files(SOURCE / "hw_test_core.vhd")

lib.add_source_files(ROOT / "testbench/hw_test_core_tb.vhd")
core_tb = lib.test_bench("hw_test_core_tb")
core_tb.add_config(name="dsp", generics=dict(pre_add_register=False))
core_tb.add_config(name="dsp_pre_add_register", generics=dict(pre_add_register=True))
core_tb.add_config(name="no_ram_output_register", generics=dict(ram_output_register=False))
core_tb.add_config(name="dsp_pre_add_register_no_ram_output_register", generics=dict(pre_add_register=True, ram_output_register=False))
core_tb.add_config(name="no_registers", generics=dict(ram_output_register=False, dsp_request_register=False))
core_tb.add_config(name="dsp_pre_add_register_no_registers", generics=dict(pre_add_register=True, ram_output_register=False, dsp_request_register=False))
core_tb.add_config(name="dsp_product_register", generics=dict(product_register=True))
core_tb.add_config(name="dsp_pre_add_and_product_registers", generics=dict(pre_add_register=True, product_register=True))
core_tb.add_config(name="spi", generics=dict(use_spi=True))
core_tb.add_config(name="spi_no_registers", generics=dict(use_spi=True, ram_output_register=False, dsp_request_register=False))

VU.main()
