LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.lut_sine_pkg.all;
    use work.lut_reciprocal_pkg.all;
    use work.lut_sqrt_pkg.all;
    use work.lut_divider_pkg.all;
    use work.full_range_sqrt_pkg.all;

-- talks to uart_test_core through its uart pins with a behavioural 8N1
-- uart, the same byte frames test_uart.py sends :
--   read  : 0x02 addr[2]          -> 7 byte response, data in the last 4
--   write : 0x04 addr[2] data[4]
entity uart_test_core_tb is
  generic (
      runner_cfg : string
      ;pre_add_register : boolean := false
      ;ram_output_register : boolean := true
      ;dsp_request_register : boolean := true
      -- the core's lut_divider table (its g_divider_* defaults)
      ;divider_index_width       : natural := 9
      ;divider_table_word_length : natural := 18
      ;divider_table_radix       : natural := 16
      ;divider_x_frac_width      : natural := 18
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

    constant divider_point_lut : reciprocal_lut_array := make_reciprocal_point_lut(divider_index_width, divider_table_word_length, divider_table_radix);
    constant divider_slope_lut : reciprocal_lut_array := make_reciprocal_slope_lut(divider_index_width, divider_table_word_length, divider_table_radix);

    -- the bytes the fpga sends, a ring buffer indexed by received_count
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
            for i in 3 to 6 loop
                data(8*(6-i)+7 downto 8*(6-i)) := received_bytes((first+i) mod received_bytes'length);
            end loop;
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

        constant sine_base       : natural := 48;
        constant reciprocal_base : natural := 64;
        constant sqrt_base       : natural := 80;
        type natural_array is array (natural range <>) of natural;
        constant calculator_bases : natural_array := (sine_base, reciprocal_base, sqrt_base);

        -- the reference functions of lut_sine_pkg, lut_reciprocal_pkg and lut_sqrt_pkg,
        -- extended to 32 bits as lut_sweep reports them
        impure function expected_result (base : natural; input : unsigned(15 downto 0)) return unsigned is
        begin
            if base = sine_base then
                return unsigned(resize(get_sine_from_quarter_wave_lut(input), 32));
            elsif base = reciprocal_base then
                return resize(get_reciprocal_from_lut(input), 32);
            else
                return resize(get_sqrt_from_lut(input), 32);
            end if;
        end expected_result;

        procedure check_single (base : natural; input : natural) is
        begin
            write_register(base + 0, input);
            write_register(base + 1, 1);
            check_register(base + 2, std_logic_vector(expected_result(base, to_unsigned(input mod 2**16, 16))));
        end check_single;

        ------------------------------
        constant divider_base   : natural := 96;
        constant quotient_radix : natural := 16;

        function galois_step (x : std_logic_vector(31 downto 0)) return std_logic_vector is
        begin
            if x(0) = '1' then
                return ('0' & x(31 downto 1)) xor x"80200003";
            else
                return '0' & x(31 downto 1);
            end if;
        end galois_step;

        procedure check_division (numerator : integer; denominator : integer) is
            constant expected : signed(31 downto 0) := lut_divide(to_signed(numerator, 32), to_signed(denominator, 32), quotient_radix
                , divider_point_lut, divider_slope_lut, divider_table_radix, divider_x_frac_width);
        begin
            write_register(divider_base + 0, numerator);
            write_register(divider_base + 1, denominator);
            write_register(divider_base + 2, 1);
            check_register(divider_base + 3, std_logic_vector(expected));
            check_register(divider_base + 4, boolean'pos(denominator = 0));
        end check_division;

        procedure check_divider_sweep (mode : natural; numerator : integer; denominator : integer; count : natural) is
            variable n, d       : std_logic_vector(31 downto 0);
            variable sum1, sum2 : unsigned(31 downto 0) := (others => '0');
            variable zeros      : natural := 0;
            variable q          : signed(31 downto 0);
            variable divisor    : signed(31 downto 0);
            variable readies    : std_logic_vector(31 downto 0);
        begin
            n := std_logic_vector(to_signed(numerator, 32));
            d := std_logic_vector(to_signed(denominator, 32));
            for i in 1 to count loop
                if mode mod 2 = 0 then
                    divisor := signed(d);
                    d := std_logic_vector(unsigned(d) + 1);
                else
                    divisor := shift_right(signed(d), to_integer(unsigned(n(4 downto 0))));
                end if;
                q := lut_divide(signed(n), divisor, quotient_radix
                    , divider_point_lut, divider_slope_lut, divider_table_radix, divider_x_frac_width);
                if divisor = 0 then
                    zeros := zeros + 1;
                end if;
                if mode mod 2 = 1 then
                    n := galois_step(n);
                    d := galois_step(d);
                end if;
                sum1 := sum1 + unsigned(q);
                sum2 := sum2 + sum1;
            end loop;
            write_register(divider_base + 0, numerator);
            write_register(divider_base + 1, denominator);
            write_register(divider_base + 7, mode);
            write_register(divider_base + 6, count mod 2**16);
            loop
                read_register(divider_base + 10, readies);
                exit when to_integer(unsigned(readies)) >= count;
            end loop;
            check_register(divider_base + 8, std_logic_vector(sum1));
            check_register(divider_base + 9, std_logic_vector(sum2));
            check_register(divider_base + 10, count);
            check_register(divider_base + 11, zeros);
        end check_divider_sweep;

        ------------------------------
        constant root_base  : natural := 112;
        constant root_radix : natural := 16;

        procedure check_root (radicand : std_logic_vector(31 downto 0)) is
        begin
            write_register(root_base + 0, radicand);
            write_register(root_base + 1, 1);
            check_register(root_base + 2, std_logic_vector(get_full_range_sqrt(unsigned(radicand), root_radix)));
        end check_root;

        procedure check_root_sweep (mode : natural; start : std_logic_vector(31 downto 0); count : natural) is
            variable x          : std_logic_vector(31 downto 0) := start;
            variable sum1, sum2 : unsigned(31 downto 0) := (others => '0');
            variable radicand   : unsigned(31 downto 0);
            variable readies    : std_logic_vector(31 downto 0);
        begin
            for i in 1 to count loop
                if mode mod 2 = 0 then
                    radicand := unsigned(x);
                    x := std_logic_vector(unsigned(x) + 1);
                else
                    radicand := shift_right(unsigned(x), to_integer(unsigned(x(4 downto 0))));
                    x := galois_step(x);
                end if;
                sum1 := sum1 + get_full_range_sqrt(radicand, root_radix);
                sum2 := sum2 + sum1;
            end loop;
            write_register(root_base + 0, start);
            write_register(root_base + 5, mode);
            write_register(root_base + 4, count mod 2**16);
            loop
                read_register(root_base + 8, readies);
                exit when to_integer(unsigned(readies)) >= count;
            end loop;
            check_register(root_base + 6, std_logic_vector(sum1));
            check_register(root_base + 7, std_logic_vector(sum2));
            check_register(root_base + 8, count);
        end check_root_sweep;

        procedure check_sweep (base : natural; start : natural; count : natural; mode : natural) is
            variable sum1, sum2 : unsigned(31 downto 0) := (others => '0');
            variable input      : unsigned(15 downto 0) := to_unsigned(start, 16);
            variable readies    : std_logic_vector(31 downto 0);
        begin
            for i in 1 to count loop
                sum1  := sum1 + expected_result(base, input);
                sum2  := sum2 + sum1;
                input := input + 1;
            end loop;
            write_register(base + 5, start);
            write_register(base + 9, mode);
            write_register(base + 4, count mod 2**16);
            loop
                read_register(base + 8, readies);
                exit when to_integer(unsigned(readies)) >= count;
            end loop;
            check_register(base + 6, std_logic_vector(sum1));
            check_register(base + 7, std_logic_vector(sum2));
            check_register(base + 8, count);
        end check_sweep;

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
        check_register(46, boolean'pos(ram_output_register));
        check_register(47, boolean'pos(dsp_request_register));
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

        -- ram request register 1 + ram 2 + dsp request register 1 +
        -- fixed_dsp 2 or 3
        for b in calculator_bases'range loop
            for i in 0 to 15 loop
                check_single(calculator_bases(b), i * 4099);
            end loop;
            check_register(calculator_bases(b) + 3, 4 + boolean'pos(ram_output_register) + boolean'pos(dsp_request_register) + boolean'pos(pre_add_register));

            check_sweep(calculator_bases(b), start => 0, count => 2**16, mode => 0);
            check_sweep(calculator_bases(b), start => 0, count => 2**16, mode => 1);
            check_sweep(calculator_bases(b), start => 65000, count => 1000, mode => 0);
        end loop;

        -- lut_divider
        check_division(1000, 3);
        check_division(-1000, 7);
        check_division(7, 0);
        check_division(integer'low, -1);
        check_division(123456789, -98765);
        check_division(1, integer'high);

        check_divider_sweep(mode => 0, numerator => 100000, denominator => -3000, count => 6000);
        check_divider_sweep(mode => 1, numerator => 16#1234567#, denominator => 16#7654321#, count => 2**16);
        check_divider_sweep(mode => 3, numerator => 16#1357#, denominator => 16#2468ace#, count => 20000);
        check_register(divider_base + 12, divider_index_width);
        check_register(divider_base + 13, divider_table_word_length);
        check_register(divider_base + 14, divider_table_radix);
        check_register(divider_base + 15, divider_x_frac_width);

        -- full_range_sqrt
        check_root(x"00000000");
        check_root(x"00010000");
        check_root(x"00020000");
        check_root(x"ffffffff");
        check_root(x"00000003");
        check_root(x"12345678");

        check_root_sweep(mode => 0, start => x"00000000", count => 6000);
        check_root_sweep(mode => 1, start => x"01234567", count => 2**16);
        check_root_sweep(mode => 3, start => x"0badcafe", count => 20000);

        test_runner_cleanup(runner);
        wait;
    end process stimulus;

    test_runner_watchdog(runner, 800 ms);
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
        received_bytes(received_count mod received_bytes'length) <= data;
        received_count <= received_count + 1;
    end process receiver;
------------------------------------------------------------------------
    u_dut : entity work.uart_test_core
    generic map (
        g_clock_divider       => g_clock_divider
        ,g_board_id           => 7
        ,g_clock_frequency_hz => 120_000_000
        ,g_dsp_pre_add_register => pre_add_register
        ,g_ram_output_register  => ram_output_register
        ,g_dsp_request_register => dsp_request_register
        ,g_divider_index_width       => divider_index_width
        ,g_divider_table_word_length => divider_table_word_length
        ,g_divider_table_radix       => divider_table_radix
        ,g_divider_x_frac_width      => divider_x_frac_width
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
