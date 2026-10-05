library ieee;
    use ieee.std_logic_1164.all;

library unisim;
    use unisim.vcomponents.all;

-- 100 MHz board oscillator -> 120 MHz logic clock
-- VCO = 100 MHz * 12 / 1 = 1200 MHz, clkout0 = 1200 MHz / 10 = 120 MHz
entity main_clocks is
    port (
        clock_100mhz : in std_logic;
        reset        : in std_logic;
        main_clock   : out std_logic;
        pll_locked   : out std_logic
    );
end entity main_clocks;

architecture rtl of main_clocks is

    signal clkfbout     : std_logic;
    signal clkfbin      : std_logic;
    signal clkout0      : std_logic;

begin

    u_pll : PLLE2_BASE
    generic map(
        BANDWIDTH          => "OPTIMIZED",
        CLKIN1_PERIOD      => 10.0,
        DIVCLK_DIVIDE      => 1,
        CLKFBOUT_MULT      => 12,
        CLKFBOUT_PHASE     => 0.0,
        CLKOUT0_DIVIDE     => 10,
        CLKOUT0_DUTY_CYCLE => 0.5,
        CLKOUT0_PHASE      => 0.0,
        STARTUP_WAIT       => "FALSE")
    port map(
        CLKIN1   => clock_100mhz,
        CLKFBIN  => clkfbin,
        CLKFBOUT => clkfbout,
        CLKOUT0  => clkout0,
        CLKOUT1  => open,
        CLKOUT2  => open,
        CLKOUT3  => open,
        CLKOUT4  => open,
        CLKOUT5  => open,
        LOCKED   => pll_locked,
        PWRDWN   => '0',
        RST      => reset);

    u_clkfb_buf : BUFG port map(I => clkfbout, O => clkfbin);
    u_clkout0_buf : BUFG port map(I => clkout0, O => main_clock);

end rtl;
