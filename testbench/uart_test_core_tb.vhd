LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

-- talks to uart_test_core through its uart pins with a behavioural 8N1
-- uart, the same byte frames test_uart.py sends :
--   read  : 0x02 addr[2]          -> 7 byte response, data in the last 4
--   write : 0x04 addr[2] data[4]
entity uart_test_core_tb is
  generic (
      runner_cfg : string
      ;pre_add_register : boolean := false
  );
end;

architecture vunit_simulation of uart_test_core_tb is

    constant clock_period    : time    := 1 ns;
    constant g_clock_divider : natural := 25;
    constant bit_time        : time    := g_clock_divider * clock_period;

    signal simulator_clock : std_logic := '0';
    signal reset           : std_logic := '1';
    signal to_fpga         : std_logic := '1';
    signal from_fpga       : std_logic;

    type byte_array is array (natural range <>) of std_logic_vector(7 downto 0);

    -- every byte the fpga sends, in order
    signal received_bytes : byte_array(0 to 1023);
    signal received_count : natural := 0;

begin

    simulator_clock <= not simulator_clock after clock_period/2.0;
    reset <= '0' after 10*clock_period;

------------------------------------------------------------------------
    stimulus : process

        procedure send_byte (data : std_logic_vector(7 downto 0)) is
        begin
            to_fpga <= '0';
            wait for bit_time;
            for i in 0 to 7 loop
                to_fpga <= data(i);
                wait for bit_time;
            end loop;
            to_fpga <= '1';
            wait for bit_time;
        end send_byte;

        procedure send_frame (frame : byte_array) is
        begin
            for i in frame'range loop
                send_byte(frame(i));
            end loop;
        end send_frame;

        function address_bytes (address : natural) return byte_array is
            constant a : std_logic_vector(15 downto 0) := std_logic_vector(to_unsigned(address, 16));
        begin
            return (a(15 downto 8), a(7 downto 0));
        end address_bytes;

        procedure write_register (address : natural; data : std_logic_vector(31 downto 0)) is
        begin
            send_frame(byte_array'(0 => x"04") & address_bytes(address)
                & byte_array'(data(31 downto 24), data(23 downto 16), data(15 downto 8), data(7 downto 0)));
        end write_register;

        procedure read_register (address : natural; data : out std_logic_vector(31 downto 0)) is
            constant first : natural := received_count;
        begin
            send_frame(byte_array'(0 => x"02") & address_bytes(address));
            if received_count < first + 7 then
                wait until received_count >= first + 7 for 200*bit_time;
            end if;
            check(received_count >= first + 7, "no response to read of register " & integer'image(address));
            data := received_bytes(first+3) & received_bytes(first+4) & received_bytes(first+5) & received_bytes(first+6);
        end read_register;

        procedure check_register (address : natural; expected : std_logic_vector(31 downto 0)) is
            variable data : std_logic_vector(31 downto 0);
        begin
            read_register(address, data);
            check_equal(data, expected, "register " & integer'image(address));
        end check_register;

        procedure write_register (address : natural; data : integer) is
        begin
            write_register(address, std_logic_vector(to_signed(data, 32)));
        end write_register;

        procedure check_register (address : natural; expected : integer) is
        begin
            check_register(address, std_logic_vector(to_signed(expected, 32)));
        end check_register;

        variable data1, data2 : std_logic_vector(31 downto 0);

    begin
        test_runner_setup(runner, runner_cfg);
        wait until reset = '0';
        wait for 10*bit_time;

        check_register(1, x"0000acdc");
        check_register(5, std_logic_vector(to_unsigned(7, 32)));
        check_register(6, std_logic_vector(to_unsigned(120_000_000, 32)));

        write_register(3, x"deadbeef");
        check_register(3, x"deadbeef");

        read_register(4, data1);
        read_register(4, data2);
        check_equal(unsigned(data2), unsigned(data1) + 1, "read counter did not count");

        read_register(7, data1);
        read_register(7, data2);
        check(unsigned(data2) > unsigned(data1), "clock counter is not running");

        for i in 16 to 31 loop
            write_register(i, std_logic_vector(to_unsigned(i * 16#01010101#, 32)));
        end loop;
        for i in 16 to 31 loop
            check_register(i, std_logic_vector(to_unsigned(i * 16#01010101#, 32)));
        end loop;
        check_register(3, x"deadbeef");

        -- fixed_dsp : (a - d) * b + c = (5 - 1) * 7 + 3 = 31
        check_register(44, 32);
        write_register(32, 5);
        write_register(33, 1);
        write_register(34, 7);
        write_register(35, 3);
        write_register(36, 0);
        write_register(37, 1); -- pre_subtract
        write_register(38, 1);
        check_register(39, 31);
        check_register(40, 0);
        check_register(41, 2 + boolean'pos(pre_add_register)); -- fixed_dsp(rtl) pipeline depth
        check_register(45, boolean'pos(pre_add_register));
        check_register(42, 1);

        -- -((a + d) * b - c) = -((5 + 1) * 7 - 3) = -39, 64 bit result
        write_register(37, 2#0110#); -- post_subtract, invert_result
        write_register(38, 1);
        check_register(39, -39);
        check_register(40, -1);

        -- 100 back to back accumulates of (a + d) * b = 6 * 7
        write_register(37, 2#1000#);
        write_register(38, 100);
        check_register(39, 4200);
        check_register(42, 100);

        -- 64 bit product : 0x40000000 * 0x40000000 = 2**60 + c 0x1_00000005
        write_register(32, 16#40000000#);
        write_register(33, 0);
        write_register(34, 16#40000000#);
        write_register(35, 5);
        write_register(36, 1);
        write_register(37, 0);
        write_register(38, 1);
        check_register(39, 5);
        check_register(40, 16#10000001#);

        -- accumulator reset
        write_register(43, 1);
        check_register(39, 0);
        check_register(40, 0);
        check_register(42, 1);

        test_runner_cleanup(runner);
        wait;
    end process stimulus;

    test_runner_watchdog(runner, 50 ms);
------------------------------------------------------------------------
    -- 8N1 receiver, runs alongside the sender since the fpga can start
    -- its response while the last stop bit of a request is still going out
    receiver : process
        variable data : std_logic_vector(7 downto 0);
    begin
        wait until from_fpga = '0';
        wait for bit_time*1.5;
        for i in 0 to 7 loop
            data(i) := from_fpga;
            wait for bit_time;
        end loop;
        check(from_fpga = '1', "missing stop bit");
        received_bytes(received_count) <= data;
        received_count <= received_count + 1;
    end process receiver;
------------------------------------------------------------------------
    u_dut : entity work.uart_test_core
    generic map (
        g_clock_divider       => g_clock_divider
        ,g_board_id           => 7
        ,g_clock_frequency_hz => 120_000_000
        ,g_dsp_pre_add_register => pre_add_register
    )
    port map (
        clock      => simulator_clock
        ,reset     => reset
        ,uart_rx   => to_fpga
        ,uart_tx   => from_fpga
        ,heartbeat => open
    );
------------------------------------------------------------------------
end vunit_simulation;
