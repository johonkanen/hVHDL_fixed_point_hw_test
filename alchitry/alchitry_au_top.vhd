------------------------------------------------------------------------
-- Alchitry Au+ (XC7A100T-1FTG256) top for hw_test_core
--
-- 100 MHz oscillator -> PLLE2 -> 120 MHz core clock
-- uart on the FT2232 channel B, 120 MHz / 24 = 5 Mbaud
-- fixed_dsp registers its pre-adder : without it the 32 bit pre-adder and the
-- cascaded 32x32 DSP48 multiply in one stage miss 120 MHz by 0.87 ns on the
-- -1 Artix-7
-- led 0 blinks at 0.5 Hz, led 7 is pll locked
------------------------------------------------------------------------
library ieee;
    use ieee.std_logic_1164.all;

    use work.fpga_interconnect_pkg.all;

entity alchitry_au_top is
    port (
        clk     : in std_logic
        ;rst_n  : in std_logic
        ;usb_rx : in std_logic
        ;usb_tx : out std_logic
        ;led    : out std_logic_vector(7 downto 0)
    );
end entity alchitry_au_top;

architecture rtl of alchitry_au_top is

    signal bus_to_communications   : fpga_interconnect_record;
    signal bus_from_communications : fpga_interconnect_record;

    signal main_clock : std_logic;
    signal pll_locked : std_logic;
    signal pll_reset  : std_logic;
    signal heartbeat  : std_logic;

begin

    pll_reset <= not rst_n;

    u_main_clocks : entity work.main_clocks
    port map (
        clock_100mhz  => clk
        ,reset        => pll_reset
        ,main_clock   => main_clock
        ,pll_locked   => pll_locked
    );

    led <= pll_locked & "000000" & heartbeat;

    u_fpga_communications : entity work.fpga_communications
    generic map (
        fpga_interconnect_pkg => work.fpga_interconnect_pkg
        ,g_clock_divider      => 24
    )
    port map (
        clock                    => main_clock
        ,uart_rx                 => usb_rx
        ,uart_tx                 => usb_tx
        ,bus_to_communications   => bus_to_communications
        ,bus_from_communications => bus_from_communications
    );

    u_core : entity work.hw_test_core
    generic map (
        g_board_id              => 1
        ,g_clock_frequency_hz   => 120_000_000
        ,g_dsp_pre_add_register => true
        ,g_ram_output_register  => false
        ,g_dsp_request_register => false
    )
    port map (
        clock                    => main_clock
        ,reset                   => not pll_locked
        ,bus_from_communications => bus_from_communications
        ,bus_to_communications   => bus_to_communications
        ,heartbeat               => heartbeat
    );

end architecture rtl;
