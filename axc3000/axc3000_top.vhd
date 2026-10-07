------------------------------------------------------------------------
-- Arrow AXC3000 (Agilex 3 A3CY100BM16AE7S) top for hw_test_core
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

    use work.fpga_interconnect_pkg.all;

entity axc3000_top is
    port (
        clk_clk        : in std_logic
        ;reset_reset_n : in std_logic
        ;uart_rxd      : in std_logic
        ;uart_txd      : out std_logic
    );
end entity axc3000_top;

architecture rtl of axc3000_top is

    signal bus_to_communications   : fpga_interconnect_record;
    signal bus_from_communications : fpga_interconnect_record;

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

    u_fpga_communications : entity work.fpga_communications
    generic map (
        fpga_interconnect_pkg => work.fpga_interconnect_pkg
        ,g_clock_divider      => 25
    )
    port map (
        clock                    => core_clock
        ,uart_rx                 => uart_rxd
        ,uart_tx                 => uart_txd
        ,bus_to_communications   => bus_to_communications
        ,bus_from_communications => bus_from_communications
    );

    u_core : entity work.hw_test_core
    generic map (
        g_board_id            => 2
        ,g_clock_frequency_hz => 120_000_000
        -- the m20k needs its output register in front of the dsp at 120 MHz
        -- (-0.41 ns without both registers), the dsp requests go unregistered
        ,g_ram_output_register => true
        ,g_dsp_request_register => false        -- the microprogram processors' rams without their output registers
        ,g_mproc_program_ram_output_register => false
        ,g_mproc_data_ram_output_register    => false
        -- the 36 bit processor's math unit : its fixed_dsps' pre-adder and
        -- product registers, its tables' ram output and dsp request registers
        ,g_mproc_math_pre_add_register     => false
        ,g_mproc_math_product_register     => false
        ,g_mproc_math_ram_output_register  => true
        ,g_mproc_math_dsp_request_register => true
        -- the 36 bit divider's first normaliser stage missed 120 MHz by
        -- 0.07 ns with 2 shifter stages
        ,g_mproc_divider_shifter_stages      => 3
    )
    port map (
        clock                    => core_clock
        ,reset                   => (not reset_reset_n) or (not pll_locked)
        ,bus_from_communications => bus_from_communications
        ,bus_to_communications   => bus_to_communications
        ,heartbeat               => open
    );

end architecture rtl;
