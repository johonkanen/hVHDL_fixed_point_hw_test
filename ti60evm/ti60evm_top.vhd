------------------------------------------------------------------------
-- Efinix Ti60F225 EVM top for uart_test_core
--
-- the periphery (uart_test.peri.xml, made by make_peri.py) has
--   25 MHz GPIOL_P_18_PLLIN0 -> main_pll (PLL_TL0) -> main_clock 120 MHz
--   uart_rx GPIOL_01 / uart_tx GPIOL_02, 1.8 V, FT4232H channel C
-- 120 MHz / 25 = 4.8 Mbaud
------------------------------------------------------------------------
library ieee;
    use ieee.std_logic_1164.all;

entity ti60evm_top is
    port (
        main_clock  : in std_logic
        ;pll_locked : in std_logic
        ;uart_rx    : in std_logic
        ;uart_tx    : out std_logic
    );
end entity ti60evm_top;

architecture rtl of ti60evm_top is

begin

    u_core : entity work.uart_test_core
    generic map (
        g_clock_divider       => 25
        ,g_board_id           => 3
        ,g_clock_frequency_hz => 120_000_000
    )
    port map (
        clock      => main_clock
        ,reset     => not pll_locked
        ,uart_rx   => uart_rx
        ,uart_tx   => uart_tx
        ,heartbeat => open
    );

end architecture rtl;
