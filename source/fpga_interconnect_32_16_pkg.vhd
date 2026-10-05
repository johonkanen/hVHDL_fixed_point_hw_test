-- fpga_interconnect with 32 bit data and 16 bit address
package fpga_interconnect_pkg is new work.fpga_interconnect_generic_pkg
    generic map(number_of_data_bits    => 32,
                number_of_address_bits => 16);
