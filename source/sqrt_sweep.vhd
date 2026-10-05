------------------------------------------------------------------------
-- sqrt_sweep - uart register test for hVHDL_fixed_point's
-- full_range_sqrt, 32 bit radicand and root
--
-- registers from g_base_address :
--
--   +0 : radicand, also the sweep start / lfsr seed               RW
--   +1 : write -> one square root of +0                           WO
--   +2 : last root                                                RO
--   +3 : clock edges from the request at the root's input to
--        its ready                                                RO
--   +4 : write N -> sweep N roots, one per clock
--        (0 is taken as 65536)                                    WO
--   +5 : sweep mode                                               RW
--        bit 0 = 0 : radicand +0, +1 per root
--        bit 0 = 1 : radicand from a 32 bit galois lfsr (x >> 1 xor
--                    0x80200003 when the low bit is 1) seeded from
--                    +0 and advanced once per root, shifted right by
--                    its own low 5 bits
--        bit 1 = 1 : irregular gaps between the requests
--   +6 : sweep sum of the roots, s1 += root                       RO
--   +7 : sweep sum of the sums, s2 += s1                          RO
--   +8 : ready pulses since the last command                      RO
------------------------------------------------------------------------
library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.full_range_sqrt_pkg.all;

entity sqrt_sweep is
    generic (
        g_base_address      : natural
        ;g_radix            : natural
        ;g_pre_add_register : boolean
    );
    port (
        clock    : in std_logic
        ;reset   : in std_logic -- synchronous, active high
        ;bus_in  : in fpga_interconnect_record
        ;bus_out : out fpga_interconnect_record
    );
end entity sqrt_sweep;

architecture rtl of sqrt_sweep is

    signal root_in  : full_range_sqrt_in_record(radicand(31 downto 0));
    signal root_out : full_range_sqrt_out_record(root(31 downto 0));

    signal radicand_register : std_logic_vector(31 downto 0) := (others => '0');
    signal sweep_mode        : std_logic_vector(31 downto 0) := (others => '0');
    signal last_root         : std_logic_vector(31 downto 0) := (others => '0');

    signal single_requested : boolean := false;
    signal sweep_left       : natural range 0 to 2**16 := 0;
    signal sweep_value      : std_logic_vector(31 downto 0) := (others => '0');
    signal gap_lfsr         : std_logic_vector(15 downto 0) := x"ace1";
    signal new_command      : boolean := false;

    signal latency_counter : unsigned(31 downto 0) := (others => '0');
    signal latency         : unsigned(31 downto 0) := (others => '0');
    signal ready_count     : unsigned(31 downto 0) := (others => '0');
    signal sum1            : unsigned(31 downto 0) := (others => '0');
    signal sum2            : unsigned(31 downto 0) := (others => '0');

    function galois_step (x : std_logic_vector(31 downto 0)) return std_logic_vector is
    begin
        if x(0) = '1' then
            return ('0' & x(31 downto 1)) xor x"80200003";
        else
            return '0' & x(31 downto 1);
        end if;
    end galois_step;

begin

    sweep_test : process (clock) is
        variable issue : boolean;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);

            connect_data_to_address(bus_in, bus_out, g_base_address + 0, radicand_register);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 2, last_root);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 3, std_logic_vector(latency));
            connect_data_to_address(bus_in, bus_out, g_base_address + 5, sweep_mode);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 6, std_logic_vector(sum1));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 7, std_logic_vector(sum2));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 8, std_logic_vector(ready_count));

            ------------------------------
            gap_lfsr <= gap_lfsr(14 downto 0) & (gap_lfsr(15) xor gap_lfsr(13) xor gap_lfsr(12) xor gap_lfsr(10));

            init_full_range_sqrt(root_in);

            issue := single_requested
                or (sweep_left > 0 and (sweep_mode(1) = '0' or gap_lfsr(0) = '1'));

            -- counters and sums restart on the first request of a command,
            -- so the latency is counted from the request at the input
            if new_command and issue then
                new_command     <= false;
                latency_counter <= (others => '0');
                ready_count     <= (others => '0');
                sum1            <= (others => '0');
                sum2            <= (others => '0');
            else
                latency_counter <= latency_counter + 1;
            end if;

            if single_requested then
                single_requested <= false;
                request_full_range_sqrt(root_in, unsigned(radicand_register));

            elsif issue then
                sweep_left <= sweep_left - 1;
                if sweep_mode(0) = '0' then
                    request_full_range_sqrt(root_in, unsigned(sweep_value));
                    sweep_value <= std_logic_vector(unsigned(sweep_value) + 1);
                else
                    request_full_range_sqrt(root_in,
                        shift_right(unsigned(sweep_value), to_integer(unsigned(sweep_value(4 downto 0)))));
                    sweep_value <= galois_step(sweep_value);
                end if;
            end if;

            ------------------------------
            -- a stale ready can not overlap the first request of a new
            -- command, uart commands are microseconds apart
            if root_out.ready_with_1 = '1' and not (new_command and issue) then
                last_root   <= std_logic_vector(root_out.root);
                ready_count <= ready_count + 1;
                sum1 <= sum1 + root_out.root;
                sum2 <= sum2 + sum1 + root_out.root;
                if ready_count = 0 then
                    latency <= latency_counter;
                end if;
            end if;

            ------------------------------
            if write_is_requested_to_address(bus_in, g_base_address + 1) then
                single_requested <= true;
                new_command      <= true;
            end if;

            if write_is_requested_to_address(bus_in, g_base_address + 4) then
                if unsigned(get_slv_data(bus_in)(15 downto 0)) = 0 then
                    sweep_left <= 2**16;
                else
                    sweep_left <= to_integer(unsigned(get_slv_data(bus_in)(15 downto 0)));
                end if;
                sweep_value <= radicand_register;
                new_command <= true;
            end if;

            if reset = '1' then
                radicand_register <= (others => '0');
                sweep_mode        <= (others => '0');
                single_requested  <= false;
                sweep_left        <= 0;
                new_command       <= false;
            end if;
        end if;
    end process sweep_test;

    u_full_range_sqrt : entity work.full_range_sqrt
    generic map (
        g_radix             => g_radix
        ,g_pre_add_register => g_pre_add_register
    )
    port map (
        clock                => clock
        ,full_range_sqrt_in  => root_in
        ,full_range_sqrt_out => root_out
    );

end architecture rtl;
