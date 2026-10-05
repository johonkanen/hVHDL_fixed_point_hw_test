------------------------------------------------------------------------
-- uart_test_core - board independent uart register test
--
-- fpga_communication uart (32 bit data, 16 bit address) with :
--
--   1      : id 0x0000ACDC                                      RO
--   2      : git hash                                           RO
--   3      : loopback register                                  RW
--   4      : read counter, incremented on every read of 4      RO
--   5      : board id (g_board_id)                              RO
--   6      : core clock frequency in Hz (g_clock_frequency_hz)  RO
--   7      : free running core clock counter                    RO
--   16..31 : register bank for pattern tests                    RW
--
-- fixed_dsp(rtl) from hVHDL_fixed_point, a/d/b g_dsp_word_length bits
-- (sign extended to 32 in the registers), c and result 2*g_dsp_word_length
-- bits split into low and high words :
--
--   32 : a                                                      RW
--   33 : d                                                      RW
--   34 : b                                                      RW
--   35 : c low word                                             RW
--   36 : c high word                                            RW
--   37 : control, bit 0 pre_subtract, 1 post_subtract,
--        2 invert_result, 3 accumulate                          RW
--   38 : write N -> N back to back fmac requests from 32..37
--        (0 is taken as 1)                                      WO
--   39 : result low word, captured on ready                     RO
--   40 : result high word                                       RO
--   41 : clock edges from the first request to its ready        RO
--   42 : ready pulses since the last command                    RO
--   43 : write -> accumulator reset request                     WO
--   44 : g_dsp_word_length                                      RO
--
-- fixed_dsp recomputes its result register on every clock, the core drives
-- init_fixed_dsp while idle so an accumulate only carries across back to
-- back requests of one burst
--
-- the board top makes the core clock and the reset, reset is active high
-- and asynchronous, it is synchronised here
------------------------------------------------------------------------
library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity uart_test_core is
    generic (
        g_clock_divider       : natural := 25 -- core clock / baud
        ;g_board_id           : natural := 0
        ;g_clock_frequency_hz : natural := 120_000_000
        ;g_dsp_word_length    : natural := 32 -- 2..32
    );
    port (
        clock      : in std_logic
        ;reset     : in std_logic
        ;uart_rx   : in std_logic
        ;uart_tx   : out std_logic
        ;heartbeat : out std_logic -- toggles at about 1 Hz
    );
end entity uart_test_core;

architecture rtl of uart_test_core is

    use work.fpga_interconnect_pkg.all;
    use work.fixed_dsp_pkg.all;
    use work.git_hash_pkg.all;

    signal reset_meta   : std_logic := '1';
    signal system_reset : std_logic := '1';

    signal bus_to_communications   : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_communications : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_top            : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_dsp            : fpga_interconnect_record := init_fpga_interconnect;

    signal loopback_register : std_logic_vector(31 downto 0) := (others => '0');
    signal read_counter      : unsigned(31 downto 0) := (others => '0');
    signal clock_counter     : unsigned(31 downto 0) := (others => '0');

    constant first_bank_address : natural := 16;
    constant bank_size          : natural := 16;
    type register_array is array (0 to bank_size-1) of std_logic_vector(31 downto 0);
    signal register_bank : register_array := (others => (others => '0'));

    ------------------------------------------------------------------
    constant dsp_n : natural := g_dsp_word_length;

    subtype dsp_in_subtype is fixed_dsp_in_record(
        a(dsp_n-1 downto 0)
        ,d(dsp_n-1 downto 0)
        ,b(dsp_n-1 downto 0)
        ,c(2*dsp_n-1 downto 0)
    );
    subtype dsp_out_subtype is fixed_dsp_out_record(
        result(2*dsp_n-1 downto 0)
    );

    signal dsp_in  : dsp_in_subtype;
    signal dsp_out : dsp_out_subtype;

    signal dsp_a       : std_logic_vector(31 downto 0) := (others => '0');
    signal dsp_d       : std_logic_vector(31 downto 0) := (others => '0');
    signal dsp_b       : std_logic_vector(31 downto 0) := (others => '0');
    signal dsp_c_low   : std_logic_vector(31 downto 0) := (others => '0');
    signal dsp_c_high  : std_logic_vector(31 downto 0) := (others => '0');
    signal dsp_control : std_logic_vector(31 downto 0) := (others => '0');

    signal dsp_result      : signed(63 downto 0) := (others => '0');
    signal requests_left   : natural range 0 to 2**16-1 := 0;
    signal reset_requested : boolean := false;
    signal new_command     : boolean := false;
    signal latency_counter : unsigned(31 downto 0) := (others => '0');
    signal dsp_latency     : unsigned(31 downto 0) := (others => '0');
    signal ready_count     : unsigned(31 downto 0) := (others => '0');
    ------------------------------------------------------------------

    signal heartbeat_counter : natural range 0 to g_clock_frequency_hz/2-1 := 0;
    signal heartbeat_state   : std_logic := '0';

begin

    reset_synchroniser : process (clock, reset) is
    begin
        if reset = '1' then
            reset_meta   <= '1';
            system_reset <= '1';
        elsif rising_edge(clock) then
            reset_meta   <= '0';
            system_reset <= reset_meta;
        end if;
    end process reset_synchroniser;

    heartbeat <= heartbeat_state;

    counters : process (clock) is
    begin
        if rising_edge(clock) then
            clock_counter <= clock_counter + 1;

            if heartbeat_counter > 0 then
                heartbeat_counter <= heartbeat_counter - 1;
            else
                heartbeat_counter <= g_clock_frequency_hz/2-1;
                heartbeat_state   <= not heartbeat_state;
            end if;
        end if;
    end process counters;

    test_registers : process (clock) is
        variable bank_index : natural range 0 to bank_size-1;
    begin
        if rising_edge(clock) then
            init_bus(bus_from_top);

            connect_read_only_data_to_address(bus_from_communications, bus_from_top, 1, x"0000acdc");
            connect_read_only_data_to_address(bus_from_communications, bus_from_top, 2, git_hash);
            connect_data_to_address(bus_from_communications, bus_from_top, 3, loopback_register);
            connect_read_only_data_to_address(bus_from_communications, bus_from_top, 4, std_logic_vector(read_counter));
            connect_read_only_data_to_address(bus_from_communications, bus_from_top, 5, std_logic_vector(to_unsigned(g_board_id, 32)));
            connect_read_only_data_to_address(bus_from_communications, bus_from_top, 6, std_logic_vector(to_unsigned(g_clock_frequency_hz, 32)));
            connect_read_only_data_to_address(bus_from_communications, bus_from_top, 7, std_logic_vector(clock_counter));

            if data_is_requested_from_address(bus_from_communications, 4) then
                read_counter <= read_counter + 1;
            end if;

            bank_index := get_address(bus_from_communications) mod bank_size;
            if write_is_requested_to_address_range(bus_from_communications, first_bank_address, first_bank_address + bank_size) then
                register_bank(bank_index) <= get_slv_data(bus_from_communications);
            end if;
            if data_is_requested_from_address_range(bus_from_communications, first_bank_address, first_bank_address + bank_size) then
                write_data_to_address(bus_from_top, 0, register_bank(bank_index));
            end if;

            bus_to_communications <= bus_from_top and bus_from_dsp;

            if system_reset = '1' then
                loopback_register     <= (others => '0');
                read_counter          <= (others => '0');
                register_bank         <= (others => (others => '0'));
                bus_to_communications <= init_fpga_interconnect;
            end if;
        end if;
    end process test_registers;

------------------------------------------------------------------------
    dsp_test : process (clock) is
    begin
        if rising_edge(clock) then
            init_bus(bus_from_dsp);

            connect_data_to_address(bus_from_communications, bus_from_dsp, 32, dsp_a);
            connect_data_to_address(bus_from_communications, bus_from_dsp, 33, dsp_d);
            connect_data_to_address(bus_from_communications, bus_from_dsp, 34, dsp_b);
            connect_data_to_address(bus_from_communications, bus_from_dsp, 35, dsp_c_low);
            connect_data_to_address(bus_from_communications, bus_from_dsp, 36, dsp_c_high);
            connect_data_to_address(bus_from_communications, bus_from_dsp, 37, dsp_control);
            connect_read_only_data_to_address(bus_from_communications, bus_from_dsp, 39, std_logic_vector(dsp_result(31 downto 0)));
            connect_read_only_data_to_address(bus_from_communications, bus_from_dsp, 40, std_logic_vector(dsp_result(63 downto 32)));
            connect_read_only_data_to_address(bus_from_communications, bus_from_dsp, 41, std_logic_vector(dsp_latency));
            connect_read_only_data_to_address(bus_from_communications, bus_from_dsp, 42, std_logic_vector(ready_count));
            connect_read_only_data_to_address(bus_from_communications, bus_from_dsp, 44, std_logic_vector(to_unsigned(dsp_n, 32)));

            ------------------------------
            init_fixed_dsp(dsp_in);

            -- counters restart on the first request of a command, so the
            -- latency is counted from the request at fixed_dsp's input
            if new_command and (requests_left > 0 or reset_requested) then
                new_command     <= false;
                latency_counter <= (others => '0');
                ready_count     <= (others => '0');
            end if;

            if requests_left > 0 then
                requests_left <= requests_left - 1;
                fmac(dsp_in
                    ,a => resize(signed(dsp_a), dsp_n)
                    ,d => resize(signed(dsp_d), dsp_n)
                    ,b => resize(signed(dsp_b), dsp_n)
                    ,c => resize(signed(std_logic_vector'(dsp_c_high & dsp_c_low)), 2*dsp_n)
                    ,pre_subtract_with_1  => dsp_control(0)
                    ,post_subtract_with_1 => dsp_control(1)
                    ,invert_result_with_1 => dsp_control(2)
                    ,accumulate_with_1    => dsp_control(3)
                );
            end if;

            if reset_requested then
                reset_requested                 <= false;
                dsp_in.request_with_1           <= '1';
                dsp_in.reset_accumulator_with_1 <= '1';
            end if;

            ------------------------------
            if dsp_out.ready_with_1 = '1' then
                dsp_result  <= resize(dsp_out.result, 64);
                ready_count <= ready_count + 1;
                if ready_count = 0 then
                    dsp_latency <= latency_counter;
                end if;
            end if;

            if not (new_command and (requests_left > 0 or reset_requested)) then
                latency_counter <= latency_counter + 1;
            end if;

            ------------------------------
            if write_is_requested_to_address(bus_from_communications, 38) then
                requests_left <= maximum(1, to_integer(unsigned(get_slv_data(bus_from_communications)(15 downto 0))));
                new_command   <= true;
            end if;

            if write_is_requested_to_address(bus_from_communications, 43) then
                reset_requested <= true;
                new_command     <= true;
            end if;

            if system_reset = '1' then
                dsp_a           <= (others => '0');
                dsp_d           <= (others => '0');
                dsp_b           <= (others => '0');
                dsp_c_low       <= (others => '0');
                dsp_c_high      <= (others => '0');
                dsp_control     <= (others => '0');
                requests_left   <= 0;
                reset_requested <= false;
                new_command     <= false;
            end if;
        end if;
    end process dsp_test;

    u_fixed_dsp : entity work.fixed_dsp(rtl)
    port map (
        clock          => clock
        ,fixed_dsp_in  => dsp_in
        ,fixed_dsp_out => dsp_out
    );

------------------------------------------------------------------------
    u_fpga_communications : entity work.fpga_communications
    generic map (
        fpga_interconnect_pkg => work.fpga_interconnect_pkg
        ,g_clock_divider      => g_clock_divider
    )
    port map (
        clock                    => clock
        ,uart_rx                 => uart_rx
        ,uart_tx                 => uart_tx
        ,bus_to_communications   => bus_to_communications
        ,bus_from_communications => bus_from_communications
    );

end architecture rtl;
