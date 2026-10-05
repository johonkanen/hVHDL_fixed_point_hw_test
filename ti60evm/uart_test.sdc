# Ti60F225 EVM uart test

# main_pll, 25 MHz on GPIOL_11_PLLIN2
create_clock -period 10.000 -name pll_feedback_clock [get_ports {pll_feedback_clock}]
create_clock -period 8.333 -name main_clock [get_ports {main_clock}]

# asynchronous uart, synchronised in the core
set_false_path -from [get_ports {uart_rx}]
set_false_path -to [get_ports {uart_tx}]
