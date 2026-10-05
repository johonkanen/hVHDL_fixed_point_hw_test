#-----------------------------------------------------------------------------
# program.tcl - load output/alchitry_au_top.bit into the Alchitry Au+ over
# JTAG (volatile, use Alchitry Loader with the .bin to write the flash)
#-----------------------------------------------------------------------------

set BITFILE [file normalize [file join [file dirname [info script]] output alchitry_au_top.bit]]

if {![file exists $BITFILE]} {
    error "$BITFILE not found, run ./build.sh first"
}

open_hw_manager
connect_hw_server
open_hw_target

set device [lindex [get_hw_devices xc7a100t*] 0]
current_hw_device $device
set_property PROGRAM.FILE $BITFILE $device
program_hw_devices $device

close_hw_target
disconnect_hw_server
close_hw_manager
