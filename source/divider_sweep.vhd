------------------------------------------------------------------------
-- divider_sweep - uart register test for hVHDL_fixed_point's lut_divider,
-- 32 bit numerator, denominator and quotient
--
-- registers from g_base_address :
--
--   +0  : numerator, also the numerator lfsr seed                RW
--   +1  : denominator, also the denominator lfsr seed            RW
--   +2  : write -> one division of +0 / +1                       WO
--   +3  : last quotient                                          RO
--   +4  : last division_by_zero                                  RO
--   +5  : clock edges from the request at the divider's input
--         to its ready                                           RO
--   +6  : write N -> sweep N divisions, one per clock
--         (0 is taken as 65536)                                  WO
--   +7  : sweep mode                                             RW
--         bit 0 = 0 : numerator +0, denominator +1, +1 per division
--         bit 0 = 1 : numerator and denominator from two 32 bit
--                     galois lfsrs (x >> 1 xor 0x80200003 when the
--                     low bit is 1) seeded from +0 and +1, advanced
--                     once per division, the denominator shifted
--                     right (arithmetic) by the numerator lfsr's
--                     low 5 bits
--         bit 1 = 1 : irregular gaps between the requests
--   +8  : sweep sum of the quotients, s1 += quotient             RO
--   +9  : sweep sum of the sums, s2 += s1                        RO
--   +10 : ready pulses since the last command                    RO
--   +11 : division_by_zero results since the last command        RO
------------------------------------------------------------------------
library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.lut_divider_pkg.all;

entity divider_sweep is
    generic (
        g_base_address      : natural
        ;g_quotient_radix   : natural
        ;g_pre_add_register : boolean
    );
    port (
        clock    : in std_logic
        ;reset   : in std_logic -- synchronous, active high
        ;bus_in  : in fpga_interconnect_record
        ;bus_out : out fpga_interconnect_record
    );
end entity divider_sweep;

architecture rtl of divider_sweep is

    signal divider_in  : lut_divider_in_record(numerator(31 downto 0), denominator(31 downto 0));
    signal divider_out : lut_divider_out_record(quotient(31 downto 0));

    signal numerator_register   : std_logic_vector(31 downto 0) := (others => '0');
    signal denominator_register : std_logic_vector(31 downto 0) := (others => '0');
    signal sweep_mode           : std_logic_vector(31 downto 0) := (others => '0');
    signal last_quotient        : std_logic_vector(31 downto 0) := (others => '0');
    signal last_division_by_zero : std_logic_vector(31 downto 0) := (others => '0');

    signal single_requested : boolean := false;
    signal sweep_left       : natural range 0 to 2**16 := 0;
    signal numerator_lfsr   : std_logic_vector(31 downto 0) := (others => '0');
    signal denominator_lfsr : std_logic_vector(31 downto 0) := (others => '0');
    signal gap_lfsr         : std_logic_vector(15 downto 0) := x"ace1";
    signal new_command      : boolean := false;

    signal latency_counter       : unsigned(31 downto 0) := (others => '0');
    signal latency               : unsigned(31 downto 0) := (others => '0');
    signal ready_count           : unsigned(31 downto 0) := (others => '0');
    signal division_by_zero_count : unsigned(31 downto 0) := (others => '0');
    signal sum1                  : unsigned(31 downto 0) := (others => '0');
    signal sum2                  : unsigned(31 downto 0) := (others => '0');

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

            connect_data_to_address(bus_in, bus_out, g_base_address + 0, numerator_register);
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, denominator_register);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 3, last_quotient);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 4, last_division_by_zero);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 5, std_logic_vector(latency));
            connect_data_to_address(bus_in, bus_out, g_base_address + 7, sweep_mode);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 8, std_logic_vector(sum1));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 9, std_logic_vector(sum2));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 10, std_logic_vector(ready_count));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 11, std_logic_vector(division_by_zero_count));

            ------------------------------
            gap_lfsr <= gap_lfsr(14 downto 0) & (gap_lfsr(15) xor gap_lfsr(13) xor gap_lfsr(12) xor gap_lfsr(10));

            init_lut_divider(divider_in);

            issue := single_requested
                or (sweep_left > 0 and (sweep_mode(1) = '0' or gap_lfsr(0) = '1'));

            -- counters and sums restart on the first request of a command,
            -- so the latency is counted from the request at the input
            if new_command and issue then
                new_command            <= false;
                latency_counter        <= (others => '0');
                ready_count            <= (others => '0');
                division_by_zero_count <= (others => '0');
                sum1                   <= (others => '0');
                sum2                   <= (others => '0');
            else
                latency_counter <= latency_counter + 1;
            end if;

            if single_requested then
                single_requested <= false;
                request_lut_division(divider_in, signed(numerator_register), signed(denominator_register));

            elsif issue then
                sweep_left <= sweep_left - 1;
                if sweep_mode(0) = '0' then
                    request_lut_division(divider_in, signed(numerator_lfsr), signed(denominator_lfsr));
                    denominator_lfsr <= std_logic_vector(unsigned(denominator_lfsr) + 1);
                else
                    request_lut_division(divider_in
                        ,signed(numerator_lfsr)
                        ,shift_right(signed(denominator_lfsr), to_integer(unsigned(numerator_lfsr(4 downto 0)))));
                    numerator_lfsr   <= galois_step(numerator_lfsr);
                    denominator_lfsr <= galois_step(denominator_lfsr);
                end if;
            end if;

            ------------------------------
            -- a stale ready can not overlap the first request of a new
            -- command, uart commands are microseconds apart
            if divider_out.ready_with_1 = '1' and not (new_command and issue) then
                last_quotient         <= std_logic_vector(divider_out.quotient);
                last_division_by_zero <= (0 => divider_out.division_by_zero, others => '0');
                ready_count <= ready_count + 1;
                if divider_out.division_by_zero = '1' then
                    division_by_zero_count <= division_by_zero_count + 1;
                end if;
                sum1 <= sum1 + unsigned(divider_out.quotient);
                sum2 <= sum2 + sum1 + unsigned(divider_out.quotient);
                if ready_count = 0 then
                    latency <= latency_counter;
                end if;
            end if;

            ------------------------------
            if write_is_requested_to_address(bus_in, g_base_address + 2) then
                single_requested <= true;
                new_command      <= true;
            end if;

            if write_is_requested_to_address(bus_in, g_base_address + 6) then
                if unsigned(get_slv_data(bus_in)(15 downto 0)) = 0 then
                    sweep_left <= 2**16;
                else
                    sweep_left <= to_integer(unsigned(get_slv_data(bus_in)(15 downto 0)));
                end if;
                -- the sweep operands start from the operand registers
                numerator_lfsr   <= numerator_register;
                denominator_lfsr <= denominator_register;
                new_command      <= true;
            end if;

            if reset = '1' then
                numerator_register   <= (others => '0');
                denominator_register <= (others => '0');
                sweep_mode           <= (others => '0');
                single_requested     <= false;
                sweep_left           <= 0;
                new_command          <= false;
            end if;
        end if;
    end process sweep_test;

    u_lut_divider : entity work.lut_divider
    generic map (
        g_quotient_radix    => g_quotient_radix
        ,g_pre_add_register => g_pre_add_register
    )
    port map (
        clock            => clock
        ,lut_divider_in  => divider_in
        ,lut_divider_out => divider_out
    );

end architecture rtl;
