# ------------------------------------------------------------------------
# Quartus Prime Pro project script - uart test for the Arrow AXC3000
# (Agilex 3 A3CY100BM16AE7S)
#
#     quartus_sh -t build.tcl compile     (or ./build.sh)
#
# "compile" generates the pll IP and runs the full flow, without it the
# script only (re)writes the project
# ------------------------------------------------------------------------

package require ::quartus::project

variable this_file_path [file dirname [file normalize [info script]]]
variable root_dir       [file dirname $this_file_path]
variable source_dir     $root_dir/source
variable com_dir        $source_dir/fpga_communication
variable fixed_dir      $source_dir/hVHDL_fixed_point
variable mproc_dir      $source_dir/hVHDL_microprogam_processor/vhdl2008

set need_to_close_project 0
if {[is_project_open]} {
    if {[string compare $quartus(project) "uart_test"]} { puts "Project uart_test is not open"; exit 1 }
} else {
    if {[project_exists uart_test]} {
        project_open -revision uart_test uart_test
    } else {
        project_new -revision uart_test uart_test
    }
    set need_to_close_project 1
}

# ---------------------------------------------------------------- device
set_global_assignment -name FAMILY "Agilex 3"
set_global_assignment -name DEVICE A3CY100BM16AE7S
set_global_assignment -name TOP_LEVEL_ENTITY axc3000_top
set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files
set_global_assignment -name MIN_CORE_JUNCTION_TEMP 0
set_global_assignment -name MAX_CORE_JUNCTION_TEMP 100
set_global_assignment -name ERROR_CHECK_FREQUENCY_DIVISOR 256
set_global_assignment -name VHDL_INPUT_VERSION VHDL_2019
set_global_assignment -name OPTIMIZATION_MODE BALANCED
set_global_assignment -name BOARD default
set_global_assignment -name USE_CONF_DONE SDM_IO16
set_global_assignment -name USE_INIT_DONE SDM_IO0

# ------------------------------------------------------------ source set
set_global_assignment -name VHDL_FILE $com_dir/hVHDL_uart/uart_rx/uart_rx_pkg.vhd
set_global_assignment -name VHDL_FILE $com_dir/hVHDL_uart/uart_tx/uart_tx_pkg.vhd
set_global_assignment -name VHDL_FILE $com_dir/serial_protocol_generic_pkg.vhd
set_global_assignment -name VHDL_FILE $com_dir/hVHDL_fpga_interconnect/fpga_interconnect_generic_pkg.vhd
set_global_assignment -name VHDL_FILE $com_dir/communications.vhd
set_global_assignment -name VHDL_FILE $source_dir/fpga_interconnect_32_16_pkg.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/fixed_dsp/fixed_dsp.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/fixed_dsp/arch_rtl_fixed_dsp.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/submodules/hVHDL_memory_library/vhdl2008/dp_ram_w_configurable_recrods.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/submodules/hVHDL_memory_library/vhdl2008/arch_rtl_dp_ram_w_configurable_records.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/lut_interpolation/lut_sine_pkg.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/sine_calculator/sine_calculator.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/lut_interpolation/lut_reciprocal_pkg.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/reciprocal_calculator/reciprocal_calculator.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/lut_interpolation/lut_sqrt_pkg.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/sqrt_calculator/sqrt_calculator.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/fixed_point_scaling/fixed_point_scaling_pkg.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/lut_divider/lut_divider.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/full_range_sqrt/full_range_sqrt.vhd
set_global_assignment -name VHDL_FILE $fixed_dir/submodules/hVHDL_memory_library/vhdl2008/mpram_w_configurable_records.vhd
set_global_assignment -name VHDL_FILE $mproc_dir/vhdl2008_microinstruction_pkg.vhd
set_global_assignment -name VHDL_FILE $mproc_dir/def_microinstruction_pkg.vhd
set_global_assignment -name VHDL_FILE $mproc_dir/microprogram_processor_pkg.vhd
set_global_assignment -name VHDL_FILE $mproc_dir/instruction_pkg.vhd
set_global_assignment -name VHDL_FILE $mproc_dir/arch_fixed_mult_add.vhd
set_global_assignment -name VHDL_FILE $mproc_dir/microprogram_sequencer.vhd
set_global_assignment -name VHDL_FILE $mproc_dir/microprogram_controller.vhd
set_global_assignment -name VHDL_FILE $source_dir/git_hash_pkg.vhd
set_global_assignment -name VHDL_FILE $source_dir/lut_sweep.vhd
set_global_assignment -name VHDL_FILE $source_dir/divider_sweep.vhd
set_global_assignment -name VHDL_FILE $source_dir/sqrt_sweep.vhd
set_global_assignment -name VHDL_FILE $source_dir/mproc_test.vhd
set_global_assignment -name VHDL_FILE $source_dir/hw_test_core.vhd
set_global_assignment -name VHDL_FILE $this_file_path/axc3000_top.vhd

# ------------------------------------------------------------------- IP
set_global_assignment -name IP_FILE $this_file_path/ip/pll_120/pll_120.ip

# ---------------------------------------------------------- constraints
set_global_assignment -name SDC_FILE $this_file_path/uart_test.sdc

# ------------------------------------------------------------------ pins
# from Arrow AXC3000 NIOSV_lab/completed_lab/NIOSV_lab.qsf
set_location_assignment PIN_A7   -to clk_clk
set_location_assignment PIN_A12  -to reset_reset_n
set_location_assignment PIN_AG23 -to uart_rxd
set_location_assignment PIN_AG24 -to uart_txd

set_instance_assignment -name IO_STANDARD "1.3-V LVCMOS" -to clk_clk       -entity axc3000_top
set_instance_assignment -name IO_STANDARD "1.3-V LVCMOS" -to reset_reset_n -entity axc3000_top
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to uart_rxd      -entity axc3000_top
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to uart_txd      -entity axc3000_top
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON   -to reset_reset_n -entity axc3000_top
set_instance_assignment -name CURRENT_STRENGTH_NEW 6MA   -to uart_txd      -entity axc3000_top

export_assignments

# ---------------------------------------------------------------- compile
if {[lsearch -exact $quartus(args) "compile"] >= 0} {

    set qgen "qsys-generate"
    if {[auto_execok $qgen] eq ""} {
        set qgen [file normalize [file join $quartus(binpath) .. sopc_builder bin qsys-generate]]
    }

    set ip_file [file join $this_file_path ip pll_120 pll_120.ip]
    puts "### qsys-generate pll_120"
    # -ignorestderr: qsys-generate prints its licence banner to stderr
    if {[catch {exec -ignorestderr $qgen $ip_file --synthesis=VHDL --part=A3CY100BM16AE7S} msg]} {
        puts $msg
        puts "ERROR: qsys-generate pll_120 failed"
        exit 1
    }

    package require ::quartus::flow
    if {[catch {execute_flow -compile} msg]} {
        puts "ERROR: compile flow failed: $msg"
        exit 1
    }
}

if {$need_to_close_project} { project_close }
