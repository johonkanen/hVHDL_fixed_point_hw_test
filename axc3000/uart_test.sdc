# Timing constraints for the AXC3000 uart test
#
# clk_clk (25 MHz) and the 120 MHz core clock are created by the SDC that
# Platform Designer generates for pll_120 and pulls in automatically

derive_clock_uncertainty

set_false_path -from [get_ports reset_reset_n]
set_false_path -from [get_ports uart_rxd]
set_false_path -to   [get_ports uart_txd]

# reset synchroniser in uart_test_core, its asynchronous preset comes from
# the reset button and pll locked
set_false_path -to [get_registers {*reset_meta* *system_reset*}]
