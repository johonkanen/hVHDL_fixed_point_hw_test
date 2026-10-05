#-----------------------------------------------------------------------------
# build.tcl - non-project Vivado build of uart_test_core for the Alchitry Au+
#
#   vivado -mode batch -source build.tcl      (or ./build.sh)
#
# outputs to output/
#-----------------------------------------------------------------------------

set PART "xc7a100tftg256-1"
set TOP  "alchitry_au_top"

set BOARD_DIR  [file normalize [file dirname [info script]]]
set ROOT_DIR   [file dirname $BOARD_DIR]
set SOURCE_DIR "$ROOT_DIR/source"
set COM_DIR    "$SOURCE_DIR/fpga_communication"
set FIXED_DIR  "$SOURCE_DIR/hVHDL_fixed_point"
set OUTPUT_DIR "$BOARD_DIR/output"

file mkdir $OUTPUT_DIR

set VHDL_SOURCES [list \
    "$COM_DIR/hVHDL_uart/uart_rx/uart_rx_pkg.vhd" \
    "$COM_DIR/hVHDL_uart/uart_tx/uart_tx_pkg.vhd" \
    "$COM_DIR/serial_protocol_generic_pkg.vhd" \
    "$COM_DIR/hVHDL_fpga_interconnect/fpga_interconnect_generic_pkg.vhd" \
    "$COM_DIR/communications.vhd" \
    "$SOURCE_DIR/fpga_interconnect_32_16_pkg.vhd" \
    "$FIXED_DIR/fixed_dsp/fixed_dsp.vhd" \
    "$FIXED_DIR/fixed_dsp/arch_rtl_fixed_dsp.vhd" \
    "$FIXED_DIR/submodules/hVHDL_memory_library/vhdl2008/dp_ram_w_configurable_recrods.vhd" \
    "$FIXED_DIR/submodules/hVHDL_memory_library/vhdl2008/arch_rtl_dp_ram_w_configurable_records.vhd" \
    "$FIXED_DIR/lut_interpolation/lut_sine_pkg.vhd" \
    "$FIXED_DIR/sine_calculator/sine_calculator.vhd" \
    "$SOURCE_DIR/git_hash_pkg.vhd" \
    "$SOURCE_DIR/uart_test_core.vhd" \
    "$BOARD_DIR/main_clocks.vhd" \
    "$BOARD_DIR/alchitry_au_top.vhd" \
]

foreach f $VHDL_SOURCES {
    if {![file exists $f]} {
        error "missing source $f, run 'git submodule update --init --recursive'"
    }
    # vhdl-2019 like the datacenter_peak_shaving builds read hVHDL_fixed_point
    read_vhdl -vhdl2019 $f
}
read_xdc "$BOARD_DIR/alchitry_au.xdc"

synth_design -top $TOP -part $PART
opt_design
place_design
phys_opt_design
route_design

report_utilization    -file "$OUTPUT_DIR/utilization.rpt"
report_timing_summary -file "$OUTPUT_DIR/timing_summary.rpt"

if {[get_property SLACK [get_timing_paths -delay_type min_max]] < 0} {
    error "timing not met, see $OUTPUT_DIR/timing_summary.rpt"
}

write_bitstream -force -bin_file "$OUTPUT_DIR/$TOP.bit"
puts "\[build] bitstream written to $OUTPUT_DIR/$TOP.bit"
