# Trion T120F324 development board, hw_test_core over spi
create_clock -period 16.667 -name main_clock [get_ports {main_clock}]
# spi_secondary's shift registers run from the spi clock, up to 30 MHz
create_clock -period 33.333 -name spi_clock [get_ports {spi_clock}]
set_clock_groups -asynchronous -group {main_clock} -group {spi_clock}
