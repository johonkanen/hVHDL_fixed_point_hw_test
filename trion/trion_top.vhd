------------------------------------------------------------------------
-- Efinix Trion T120F324 development board top for hw_test_core
--
-- the periphery (uart_test.peri.xml, made by make_peri.py) has
--   30 MHz GPIOL_15 -> main_pll (PLL_BL0) -> main_clock 60 MHz
--   spi from the FT2232H channel A : clock GPIOL_01, chip select GPIOL_00,
--   data in GPIOL_08, data out GPIOL_09 ; the board has no uart to the fpga
--   user_led[3:0]
-- led 0 blinks at 0.5 Hz, led 1 is pll locked
------------------------------------------------------------------------
library ieee;
    use ieee.std_logic_1164.all;

    use work.fpga_interconnect_pkg.all;

entity trion_top is
    port (
        main_clock    : in std_logic
        ;pll_locked   : in std_logic
        ;spi_clock    : in std_logic
        ;spi_cs_in    : in std_logic
        ;spi_data_in  : in std_logic
        ;spi_data_out : out std_logic
        ;user_led     : out std_logic_vector(3 downto 0)
    );
end entity trion_top;

architecture rtl of trion_top is

    signal bus_to_communications   : fpga_interconnect_record;
    signal bus_from_communications : fpga_interconnect_record;

    signal heartbeat : std_logic;

begin

    user_led <= "00" & pll_locked & heartbeat;

    u_fpga_spi_communications : entity work.fpga_spi_communications
    generic map (
        fpga_interconnect_pkg => work.fpga_interconnect_pkg
    )
    port map (
        clock                    => main_clock
        ,spi_clock               => spi_clock
        ,spi_cs_in               => spi_cs_in
        ,spi_data_in             => spi_data_in
        ,spi_data_out            => spi_data_out
        ,bus_to_communications   => bus_to_communications
        ,bus_from_communications => bus_from_communications
    );

    u_core : entity work.hw_test_core
    generic map (
        g_board_id               => 4
        ,g_clock_frequency_hz   => 60_000_000
        -- the trion's fabric is slower than titanium's : every pipeline
        -- register on to start with
        ,g_dsp_pre_add_register => true
        ,g_ram_output_register  => true
        ,g_dsp_request_register => true        -- the microprogram processors' rams without their output registers
        ,g_mproc_program_ram_output_register => false
        ,g_mproc_data_ram_output_register    => false
        -- the 36 bit processor's math unit : its fixed_dsps' pre-adder and
        -- product registers, its tables' ram output and dsp request registers
        ,g_mproc_math_pre_add_register     => true
        ,g_mproc_math_product_register     => false
        ,g_mproc_math_ram_output_register  => true
        ,g_mproc_math_dsp_request_register => true
    )
    port map (
        clock                    => main_clock
        ,reset                   => not pll_locked
        ,bus_from_communications => bus_from_communications
        ,bus_to_communications   => bus_to_communications
        ,heartbeat               => heartbeat
    );

end architecture rtl;
