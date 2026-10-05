------------------------------------------------------------------------
-- lut_sweep - uart register test for a fully pipelined 16 bit in, 16 bit
-- out calculator (sine_calculator, reciprocal_calculator, ...)
--
-- registers from g_base_address :
--
--   +0 : input                                                  RW
--   +1 : write -> one request for the input in +0               WO
--   +2 : last result, sign or zero extended (g_signed_result)   RO
--   +3 : clock edges from the request at the calculator's
--        input to its ready                                     RO
--   +4 : write N -> sweep N inputs from +5 upwards, one per
--        clock (0 is taken as 65536)                            WO
--   +5 : sweep start input                                      RW
--   +6 : sweep sum of the results, s1 += result                 RO
--   +7 : sweep sum of the sums, s2 += s1                        RO
--   +8 : ready pulses since the last command                    RO
--   +9 : sweep mode, 0 back to back, 1 irregular gaps           RW
--
-- request_value / request_with_1 are registered and go straight to the
-- calculator, result / ready_with_1 come straight back from it
------------------------------------------------------------------------
library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;

entity lut_sweep is
    generic (
        g_base_address   : natural
        ;g_signed_result : boolean
    );
    port (
        clock           : in std_logic
        ;reset          : in std_logic -- synchronous, active high
        ;bus_in         : in fpga_interconnect_record
        ;bus_out        : out fpga_interconnect_record
        ;request_value  : out unsigned(15 downto 0)
        ;request_with_1 : out std_logic
        ;result         : in std_logic_vector(15 downto 0)
        ;ready_with_1   : in std_logic
    );
end entity lut_sweep;

architecture rtl of lut_sweep is

    signal input_register   : std_logic_vector(31 downto 0) := (others => '0');
    signal sweep_start      : std_logic_vector(31 downto 0) := (others => '0');
    signal sweep_mode       : std_logic_vector(31 downto 0) := (others => '0');
    signal last_result      : std_logic_vector(31 downto 0) := (others => '0');
    signal single_requested : boolean := false;
    signal sweep_left       : natural range 0 to 2**16 := 0;
    signal sweep_value      : unsigned(15 downto 0) := (others => '0');
    signal sweep_lfsr       : std_logic_vector(15 downto 0) := x"ace1";
    signal new_command      : boolean := false;
    signal latency_counter  : unsigned(31 downto 0) := (others => '0');
    signal latency          : unsigned(31 downto 0) := (others => '0');
    signal ready_count      : unsigned(31 downto 0) := (others => '0');
    signal sum1             : unsigned(31 downto 0) := (others => '0');
    signal sum2             : unsigned(31 downto 0) := (others => '0');

    function to_word (data : std_logic_vector(15 downto 0)) return std_logic_vector is
    begin
        if g_signed_result then
            return std_logic_vector(resize(signed(data), 32));
        else
            return std_logic_vector(resize(unsigned(data), 32));
        end if;
    end to_word;

begin

    sweep_test : process (clock) is
        variable issue : boolean;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);

            connect_data_to_address(bus_in, bus_out, g_base_address + 0, input_register);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 2, last_result);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 3, std_logic_vector(latency));
            connect_data_to_address(bus_in, bus_out, g_base_address + 5, sweep_start);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 6, std_logic_vector(sum1));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 7, std_logic_vector(sum2));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 8, std_logic_vector(ready_count));
            connect_data_to_address(bus_in, bus_out, g_base_address + 9, sweep_mode);

            ------------------------------
            -- 16 bit maximal length lfsr, its low bit gates the requests
            -- of a gapped sweep
            sweep_lfsr <= sweep_lfsr(14 downto 0) & (sweep_lfsr(15) xor sweep_lfsr(13) xor sweep_lfsr(12) xor sweep_lfsr(10));

            request_with_1 <= '0';

            issue := single_requested
                or (sweep_left > 0 and (sweep_mode(0) = '0' or sweep_lfsr(0) = '1'));

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
                request_with_1   <= '1';
                request_value    <= unsigned(input_register(15 downto 0));
            elsif issue then
                sweep_left     <= sweep_left - 1;
                sweep_value    <= sweep_value + 1;
                request_with_1 <= '1';
                request_value  <= sweep_value;
            end if;

            ------------------------------
            -- a stale ready can not overlap the first request of a new
            -- command, uart commands are microseconds apart
            if ready_with_1 = '1' and not (new_command and issue) then
                last_result <= to_word(result);
                ready_count <= ready_count + 1;
                sum1 <= sum1 + unsigned(to_word(result));
                sum2 <= sum2 + sum1 + unsigned(to_word(result));
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
                sweep_value <= unsigned(sweep_start(15 downto 0));
                new_command <= true;
            end if;

            if reset = '1' then
                input_register   <= (others => '0');
                sweep_start      <= (others => '0');
                sweep_mode       <= (others => '0');
                single_requested <= false;
                sweep_left       <= 0;
                new_command      <= false;
                request_with_1   <= '0';
            end if;
        end if;
    end process sweep_test;

end architecture rtl;
