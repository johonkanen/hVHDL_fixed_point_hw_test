------------------------------------------------------------------------
-- Arrow AXC3000 (Agilex 3 A3CY100BM16AE7S) top for uart_test_core
--
--   clk_clk       PIN_A7    1.3-V LVCMOS   25 MHz oscillator
--   reset_reset_n PIN_A12   1.3-V LVCMOS   active low, weak pull-up
--   uart_rxd      PIN_AG23  3.3-V LVCMOS
--   uart_txd      PIN_AG24  3.3-V LVCMOS
--
-- 25 MHz -> pll_120 IOPLL -> 120 MHz core clock
-- uart on the USB Blaster III, 120 MHz / 25 = 4.8 Mbaud
------------------------------------------------------------------------
library ieee;
    use ieee.std_logic_1164.all;

entity axc3000_top is
    port (
        clk_clk        : in std_logic
        ;reset_reset_n : in std_logic
        ;uart_rxd      : in std_logic
        ;uart_txd      : out std_logic
    );
end entity axc3000_top;

architecture rtl of axc3000_top is

    component pll_120 is
        port (
            rst       : in std_logic := 'X'
            ;refclk   : in std_logic := 'X'
            ;locked   : out std_logic
            ;outclk_0 : out std_logic
        );
    end component pll_120;

    signal core_clock : std_logic;
    signal pll_locked : std_logic;

begin

    u_pll : component pll_120
    port map (
        rst       => not reset_reset_n
        ,refclk   => clk_clk
        ,locked   => pll_locked
        ,outclk_0 => core_clock
    );

    u_core : entity work.uart_test_core
    generic map (
        g_clock_divider       => 25
        ,g_board_id           => 2
        ,g_clock_frequency_hz => 120_000_000
        ,g_ram_output_register => false
    )
    port map (
        clock      => core_clock
        ,reset     => (not reset_reset_n) or (not pll_locked)
        ,uart_rx   => uart_rxd
        ,uart_tx   => uart_txd
        ,heartbeat => open
    );

end architecture rtl;
