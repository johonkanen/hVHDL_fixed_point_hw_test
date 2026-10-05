# Alchitry Au+ (XC7A100T-1FTG256), pins from alchitry_au_sch.pdf

# 100 MHz oscillator
set_property -dict {PACKAGE_PIN N14 IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 10.000 -name clk_100mhz [get_ports clk]

# reset button, active low
set_property -dict {PACKAGE_PIN P6 IOSTANDARD LVCMOS33} [get_ports rst_n]

# FT2232 uart
set_property -dict {PACKAGE_PIN P15 IOSTANDARD LVCMOS33} [get_ports usb_rx]
set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS33} [get_ports usb_tx]

# leds
set_property -dict {PACKAGE_PIN K13 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN K12 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN L14 IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN L13 IOSTANDARD LVCMOS33} [get_ports {led[3]}]
set_property -dict {PACKAGE_PIN M16 IOSTANDARD LVCMOS33} [get_ports {led[4]}]
set_property -dict {PACKAGE_PIN M14 IOSTANDARD LVCMOS33} [get_ports {led[5]}]
set_property -dict {PACKAGE_PIN M12 IOSTANDARD LVCMOS33} [get_ports {led[6]}]
set_property -dict {PACKAGE_PIN N16 IOSTANDARD LVCMOS33} [get_ports {led[7]}]

# asynchronous io, not timed
set_false_path -from [get_ports {rst_n usb_rx}]
set_false_path -to [get_ports {usb_tx led[*]}]

# configuration
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
