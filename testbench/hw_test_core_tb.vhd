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

-- talks to hw_test_core through fpga_communications' uart pins with a
-- behavioural 8N1 uart (or fpga_spi_communications' pins, use_spi), the same byte frames test_uart.py sends :
--   read  : 0x02 addr[2]          -> 7 byte response, data in the last 4
--   write : 0x04 addr[2] data[4]
entity hw_test_core_tb is
  generic (
      runner_cfg : string
      -- the link : the uart, or spi (mode 0, spi_half_period clocks per
      -- half spi clock period) like on boards without a uart
      ;use_spi         : boolean  := false
      ;spi_half_period : positive := 4
      ;pre_add_register : boolean := false
      ;product_register : boolean := false
      ;mproc_program_ram_register : boolean := true
      ;mproc_data_ram_register    : boolean := true
      ;mproc_divider_shifter_stages : positive := 2
      ;ram_output_register : boolean := true
      ;dsp_request_register : boolean := true
      -- the core's lut_divider table (its g_divider_* defaults)
      ;divider_index_width       : natural := 9
      ;divider_table_word_length : natural := 18
      ;divider_table_radix       : natural := 16
      ;divider_x_frac_width      : natural := 18
      -- the core's full_range_sqrt table (its g_root_* defaults)
      ;root_index_width       : natural := 9
      ;root_table_word_length : natural := 18
      ;root_table_radix       : natural := 17
      ;root_x_frac_width      : natural := 18
  );
end;

architecture vunit_simulation of hw_test_core_tb is

    constant clock_period    : time    := 1 ns;
    constant g_clock_divider : natural := 25;
    constant bit_time        : time    := g_clock_divider * clock_period;

    signal simulator_clock : std_logic := '0';
    signal reset           : std_logic := '1';
    signal to_fpga         : std_logic := '1';
    signal from_fpga       : std_logic;

    signal spi_clock    : std_logic := '0';
    signal spi_cs_in    : std_logic := '1';
    signal spi_data_in  : std_logic := '0';
    signal spi_data_out : std_logic;

    signal bus_to_communications   : work.fpga_interconnect_pkg.fpga_interconnect_record;
    signal bus_from_communications : work.fpga_interconnect_pkg.fpga_interconnect_record;

    type byte_array is array (natural range <>) of std_logic_vector(7 downto 0);

    constant divider_point_lut : reciprocal_lut_array := make_reciprocal_point_lut(divider_index_width, divider_table_word_length, divider_table_radix);
    constant divider_slope_lut : reciprocal_lut_array := make_reciprocal_slope_lut(divider_index_width, divider_table_word_length, divider_table_radix);
    constant root_point_lut : sqrt_lut_array := make_sqrt_point_lut(root_index_width, root_table_word_length, root_table_radix);
    constant root_slope_lut : sqrt_lut_array := make_sqrt_slope_lut(root_index_width, root_table_word_length, root_table_radix);

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

        -- one spi exchange, chip select low throughout, mode 0, msb first
        procedure spi_exchange (tx : byte_array; rx : out byte_array) is
            constant half : time := spi_half_period * clock_period;
            variable byte : std_logic_vector(7 downto 0);
        begin
            spi_cs_in <= '0';
            wait for half;
            for i in tx'range loop
                for b in 7 downto 0 loop
                    spi_data_in <= tx(i)(b);
                    wait for half;
                    byte(b)   := spi_data_out;
                    spi_clock <= '1';
                    wait for half;
                    spi_clock <= '0';
                end loop;
                rx(i) := byte;
            end loop;
            wait for half;
            spi_cs_in <= '1';
            wait for 4*half;
        end spi_exchange;

        procedure write_register (address : natural; data : std_logic_vector(31 downto 0)) is
            constant frame : byte_array := byte_array'(0 => x"04") & address_bytes(address)
                & byte_array'(data(31 downto 24), data(23 downto 16), data(15 downto 8), data(7 downto 0));
            variable rx : byte_array(frame'range);
        begin
            if use_spi then
                spi_exchange(frame, rx);
            else
                send_frame(frame);
            end if;
        end write_register;

        procedure read_register (address : natural; data : out std_logic_vector(31 downto 0)) is
            constant first   : natural := received_count;
            -- spi : the command, then zeros that clock the response out ; the
            -- response is the 7 byte frame starting at the first nonzero byte
            -- after the first one out (0xff)
            constant command : byte_array := byte_array'(0 => x"02") & address_bytes(address);
            constant tx      : byte_array := command & byte_array'(0 to 23 => x"00");
            variable rx      : byte_array(tx'range);
            variable start   : integer := -1;
        begin
            if use_spi then
                spi_exchange(tx, rx);
                for i in 1 to rx'high loop
                    if rx(i) /= x"00" then
                        start := i;
                        exit;
                    end if;
                end loop;
                check(start >= 0 and start + 6 <= rx'high, "no spi response to read of register " & integer'image(address));
                data := rx(start+3) & rx(start+4) & rx(start+5) & rx(start+6);
                return;
            end if;
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
        variable x, y         : std_logic_vector(31 downto 0);
        variable i_state, u_state : std_logic_vector(31 downto 0);
        type word36_array is array (natural range <>) of signed(35 downto 0);
        variable operands36 : word36_array(64 to 87);
        variable quotient36, product36 : signed(35 downto 0);
        type word_array is array (natural range <>) of std_logic_vector(31 downto 0);
        variable operands     : word_array(64 to 87);

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
            check_register(root_base + 2, std_logic_vector(get_full_range_sqrt(unsigned(radicand), root_radix
                , root_point_lut, root_slope_lut, root_table_radix, root_x_frac_width)));
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
                sum1 := sum1 + get_full_range_sqrt(radicand, root_radix
                    , root_point_lut, root_slope_lut, root_table_radix, root_x_frac_width);
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

        -- mproc_test : fixed_mult_add's a * b + c * 2**radix, bits radix + 31
        -- downto radix ; the sums and differences wrap to 32 bits
        constant mproc_base     : natural := 128;
        constant mproc_ram_base : natural := 256;
        constant mproc_radix    : natural := 20;
        -- both processors' result latency, the run times follow it
        constant mproc_latency  : natural := work.execution_unit_pkg.fixed_point_result_latency(pre_add_register, product_register, mproc_data_ram_register);
        constant mproc_slots    : natural := work.microprogram_interface_pkg.jump_delay_slots(mproc_program_ram_register);

        function mult_add (a, b, c : std_logic_vector(31 downto 0)) return std_logic_vector is
            variable result : signed(63 downto 0);
        begin
            result := signed(a) * signed(b) + shift_left(resize(signed(c), 64), mproc_radix);
            return std_logic_vector(result(mproc_radix + 31 downto mproc_radix));
        end mult_add;

        function mult_sub (a, b, c : std_logic_vector(31 downto 0)) return std_logic_vector is
            variable result : signed(63 downto 0);
        begin
            result := signed(a) * signed(b) - shift_left(resize(signed(c), 64), mproc_radix);
            return std_logic_vector(result(mproc_radix + 31 downto mproc_radix));
        end mult_sub;

        function sum (a, b : std_logic_vector(31 downto 0)) return std_logic_vector is
        begin
            return std_logic_vector(signed(a) + signed(b));
        end sum;

        function minus (a : std_logic_vector(31 downto 0)) return std_logic_vector is
        begin
            return std_logic_vector(-signed(a));
        end minus;

        -- run a program and check how many clocks it took to ready
        procedure run_program (start : natural; clocks : natural) is
            variable data : std_logic_vector(31 downto 0);
        begin
            write_register(mproc_base, start);
            write_register(mproc_base + 1, 1);
            for i in 1 to 100 loop
                read_register(mproc_base + 2, data);
                exit when data = x"00000000";
            end loop;
            check_register(mproc_base + 2, 0);
            check_register(mproc_base + 3, 1);
            check_register(mproc_base + 4, clocks);
        end run_program;

        function to_fixed (x : real) return std_logic_vector is
        begin
            return std_logic_vector(to_signed(integer(x * 2.0**mproc_radix), 32));
        end to_fixed;

        -- the 36 bit processor : data through the high and low windows
        constant mproc36_base      : natural := 144;
        constant mproc36_ram       : natural := 512;
        constant mproc36_ram_high  : natural := 768;
        constant mproc36_radix     : natural := 24;
        subtype word36 is signed(35 downto 0);

        procedure write_word36 (address : natural; value : word36) is
        begin
            write_register(mproc36_ram_high + address, std_logic_vector(resize(value(35 downto 32), 32)));
            write_register(mproc36_ram + address, std_logic_vector(value(31 downto 0)));
        end write_word36;

        procedure check_word36 (address : natural; expected : word36) is
            variable low, high : std_logic_vector(31 downto 0);
        begin
            read_register(mproc36_ram + address, low);
            read_register(mproc36_ram_high + address, high);
            check_equal(std_logic_vector(resize(signed(high), 4)) & low, std_logic_vector(expected),
                "36 bit data ram " & integer'image(address));
        end check_word36;

        function mult_add36 (a, b, c : word36; subtract : boolean := false) return word36 is
            variable result : signed(71 downto 0);
        begin
            if subtract then
                result := a * b - shift_left(resize(c, 72), mproc36_radix);
            else
                result := a * b + shift_left(resize(c, 72), mproc36_radix);
            end if;
            return result(mproc36_radix + 35 downto mproc36_radix);
        end mult_add36;

        -- n steps of mproc_test's boost converter model, program 128
        procedure boost_steps (n : natural; i, u : inout std_logic_vector(31 downto 0);
            vin, d, load, r, i_gain, u_gain : std_logic_vector(31 downto 0)) is
            variable vl, ic : std_logic_vector(31 downto 0);
        begin
            for k in 1 to n loop
                vl := mult_add(minus(d), u, vin);
                ic := mult_sub(d, i, load);
                vl := mult_add(minus(r), i, vl);
                u  := mult_add(ic, u_gain, u);
                i  := mult_add(vl, i_gain, i);
            end loop;
        end boost_steps;

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
        check_register(41, 2 + boolean'pos(pre_add_register) + boolean'pos(product_register)); -- fixed_dsp(rtl) pipeline depth
        check_register(125, boolean'pos(product_register));
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
            check_register(calculator_bases(b) + 3, 4 + boolean'pos(ram_output_register) + boolean'pos(dsp_request_register) + boolean'pos(pre_add_register) + boolean'pos(product_register));

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
        check_register(root_base + 9, root_index_width);
        check_register(root_base + 10, root_table_word_length);
        check_register(root_base + 11, root_table_radix);
        check_register(root_base + 12, root_x_frac_width);

        -- microprogram processor, program 0 : one of each command
        check_register(mproc_base + 5, mproc_radix);
        check_register(mproc_base + 12, mproc_latency);
        check_register(mproc_base + 13, mproc_slots);
        write_register(mproc_ram_base + 1, x"5a5a5a5a"); -- program 0 overwrites it
        check_register(mproc_ram_base + 1, x"5a5a5a5a");
        x := x"1234abcd";
        for i in 64 to 87 loop
            x := galois_step(x);
            operands(i) := std_logic_vector(shift_right(signed(x), 4));
            write_register(mproc_ram_base + i, operands(i));
        end loop;
        check_register(mproc_ram_base + 70, operands(70));
        run_program(0, clocks => 12 + mproc_slots + mproc_latency);
        check_register(mproc_ram_base + 1, mult_add(operands(64), operands(65), operands(66)));
        check_register(mproc_ram_base + 2, mult_sub(operands(67), operands(68), operands(69)));
        check_register(mproc_ram_base + 3, mult_add(minus(operands(70)), operands(71), operands(72)));
        check_register(mproc_ram_base + 4, mult_sub(minus(operands(73)), operands(74), operands(75)));
        check_register(mproc_ram_base + 5, mult_add(sum(operands(76), operands(77)), operands(78), x"00000000"));
        check_register(mproc_ram_base + 6, mult_add(sum(operands(79), minus(operands(80))), operands(81), x"00000000"));
        check_register(mproc_ram_base + 7, mult_add(sum(operands(82), minus(operands(83))), operands(84), operands(83)));
        check_register(mproc_ram_base + 8, sum(sum(operands(85), operands(86)), operands(87)));

        -- program 32 : 100 rounds of the low pass filter y += (u - y) * g
        y := x"00000000";
        write_register(mproc_ram_base + 96, y);
        write_register(mproc_ram_base + 97, 3 * 2**mproc_radix);    -- u = 3.0
        write_register(mproc_ram_base + 98, 2**mproc_radix / 20);              -- g = 0.05
        for i in 1 to 100 loop
            y := mult_add(sum(std_logic_vector(to_signed(3 * 2**mproc_radix, 32)), minus(y))
                , std_logic_vector(to_signed(2**mproc_radix / 20, 32)), y);
        end loop;
        run_program(32, clocks => 4 + mproc_slots + 100 * mproc_latency);
        check_register(mproc_ram_base + 96, y);

        -- program 128 : one boost converter step from its initial data
        check_register(mproc_ram_base + 100, to_fixed(20.0));
        check_register(mproc_ram_base + 107, to_fixed(12.0));
        i_state := to_fixed(0.0);
        u_state := to_fixed(12.0);
        boost_steps(1, i_state, u_state, to_fixed(20.0), to_fixed(0.8), to_fixed(0.0), to_fixed(0.8), to_fixed(0.7 / 3.0), to_fixed(0.7 / 3.0));
        run_program(128, clocks => 3 + mproc_slots + 3 * mproc_latency);
        check_register(mproc_ram_base + 106, i_state);
        check_register(mproc_ram_base + 107, u_state);

        -- in the background with new data, replayed for the runs it made
        write_register(mproc_ram_base + 100, to_fixed(10.0));
        write_register(mproc_ram_base + 101, to_fixed(0.5));
        write_register(mproc_ram_base + 102, to_fixed(0.25));
        i_state := to_fixed(1.0);
        u_state := to_fixed(5.0);
        write_register(mproc_ram_base + 106, i_state);
        write_register(mproc_ram_base + 107, u_state);
        write_register(mproc_base + 7, 60);
        write_register(mproc_base + 6, 1);
        for k in 1 to 20 loop
            read_register(mproc_base + 8, data1);
            exit when unsigned(data1) >= 20;
        end loop;
        write_register(mproc_base + 6, 0);
        for k in 1 to 20 loop
            read_register(mproc_base + 2, data1);
            exit when data1 = x"00000000";
        end loop;
        read_register(mproc_base + 8, data1);
        check(unsigned(data1) >= 20, "background runs " & integer'image(to_integer(unsigned(data1))));
        boost_steps(to_integer(unsigned(data1)), i_state, u_state, to_fixed(10.0), to_fixed(0.5), to_fixed(0.25), to_fixed(0.8), to_fixed(0.7 / 3.0), to_fixed(0.7 / 3.0));
        check_register(mproc_ram_base + 106, i_state);
        check_register(mproc_ram_base + 107, u_state);
        -- the 36 bit processor, program 0, operands over the full 36 bits
        check_register(mproc36_base + 5, mproc36_radix);
        check_register(mproc36_base + 9, 36);
        check_register(mproc36_base + 10, 36);
        check_register(mproc36_base + 11, 256);
        check_register(mproc_base + 11, 128);
        x := x"0badcafe";
        for k in 64 to 87 loop
            x := galois_step(x);
            y := galois_step(x);
            operands36(k) := signed(y(3 downto 0)) & signed(x);
            write_word36(k, operands36(k));
        end loop;
        operands36(70) := (35 => '1', others => '0'); -- -2**35, the pre-adder wraps
        write_word36(70, operands36(70));
        check_word36(70, operands36(70));
        write_register(mproc36_base, 0);
        write_register(mproc36_base + 1, 1);
        for k in 1 to 20 loop
            read_register(mproc36_base + 2, data1);
            exit when data1 = x"00000000";
        end loop;
        check_register(mproc36_base + 3, 1);
        check_register(mproc36_base + 4, 12 + mproc_slots + mproc_latency);
        check_word36(1, mult_add36(operands36(64), operands36(65), operands36(66)));
        check_word36(2, mult_add36(operands36(67), operands36(68), operands36(69), subtract => true));
        check_word36(3, mult_add36(-operands36(70), operands36(71), operands36(72)));
        check_word36(4, mult_add36(-operands36(73), operands36(74), operands36(75), subtract => true));
        check_word36(5, mult_add36(operands36(76) + operands36(77), operands36(78), (others => '0')));
        check_word36(6, mult_add36(operands36(79) - operands36(80), operands36(81), (others => '0')));
        check_word36(7, mult_add36(operands36(82) - operands36(83), operands36(84), operands36(83)));
        check_word36(8, operands36(85) + operands36(86) + operands36(87));

        -- program 192 : 8 bit address fields reach operands above 127
        for k in 200 to 205 loop
            x := galois_step(x);
            y := galois_step(x);
            write_word36(k, signed(y(3 downto 0)) & signed(x));
            operands36(64 + k - 200) := signed(y(3 downto 0)) & signed(x);
        end loop;
        write_register(mproc36_base, 192);
        write_register(mproc36_base + 1, 1);
        for k in 1 to 20 loop
            read_register(mproc36_base + 2, data1);
            exit when data1 = x"00000000";
        end loop;
        check_word36(250, mult_add36(operands36(64), operands36(65), operands36(66)));
        check_word36(251, mult_add36(operands36(67), operands36(68), operands36(69), subtract => true));

        -- program 224 : the math unit, 112 <- 110 / 111,
        -- 113 <- 112 * 114 + 115, 116 <- 113 / 111
        check_register(mproc_base + 14, 0);
        check_register(mproc36_base + 14, work.execution_unit_pkg.fixed_math_result_latency(
            pre_add_register, product_register, mproc_data_ram_register, mproc_divider_shifter_stages));
        write_word36(110, to_signed(5 * 2**23, 36));       --  2.5 at radix 24
        write_word36(111, to_signed(-3 * 2**22, 36));      -- -0.75
        write_word36(114, signed(x(3 downto 0)) & signed(y));
        write_word36(115, to_signed(7 * 2**20, 36));
        write_register(mproc36_base, 224);
        write_register(mproc36_base + 1, 1);
        for k in 1 to 20 loop
            read_register(mproc36_base + 2, data1);
            exit when data1 = x"00000000";
        end loop;
        check_register(mproc36_base + 3, 1);
        quotient36 := lut_divide(to_signed(5 * 2**23, 36), to_signed(-3 * 2**22, 36), mproc36_radix
            , divider_point_lut, divider_slope_lut, divider_table_radix, divider_x_frac_width);
        check_word36(112, quotient36);
        product36 := mult_add36(quotient36, signed(x(3 downto 0)) & signed(y), to_signed(7 * 2**20, 36));
        check_word36(113, product36);
        check_word36(116, lut_divide(product36, to_signed(-3 * 2**22, 36), mproc36_radix
            , divider_point_lut, divider_slope_lut, divider_table_radix, divider_x_frac_width));

        -- program 288 : 118 <- sqrt(117), 119 <- 117 / 118
        write_word36(117, signed(x(3 downto 0)) & signed(y) and x"7ffffffff");
        write_register(mproc36_base, 288);
        write_register(mproc36_base + 1, 1);
        for k in 1 to 20 loop
            read_register(mproc36_base + 2, data1);
            exit when data1 = x"00000000";
        end loop;
        check_register(mproc36_base + 3, 1);
        quotient36 := signed(get_full_range_sqrt(unsigned(signed(x(3 downto 0)) & signed(y) and x"7ffffffff"), mproc36_radix
            , root_point_lut, root_slope_lut, root_table_radix, root_x_frac_width));
        check_word36(118, quotient36);
        check_word36(119, lut_divide(signed(x(3 downto 0)) & signed(y) and x"7ffffffff", quotient36, mproc36_radix
            , divider_point_lut, divider_slope_lut, divider_table_radix, divider_x_frac_width));

        info("boost converter after " & integer'image(to_integer(unsigned(data1))) & " background steps : i "
            & real'image(real(to_integer(signed(i_state))) / 2.0**mproc_radix) & " u "
            & real'image(real(to_integer(signed(u_state))) / 2.0**mproc_radix));

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
    uart_link : if not use_spi generate
        u_fpga_communications : entity work.fpga_communications
        generic map (
            fpga_interconnect_pkg => work.fpga_interconnect_pkg
            ,g_clock_divider      => g_clock_divider
        )
        port map (
            clock                    => simulator_clock
            ,uart_rx                 => to_fpga
            ,uart_tx                 => from_fpga
            ,bus_to_communications   => bus_to_communications
            ,bus_from_communications => bus_from_communications
        );
    end generate;

    spi_link : if use_spi generate
        from_fpga <= '1';

        u_fpga_spi_communications : entity work.fpga_spi_communications
        generic map (
            fpga_interconnect_pkg => work.fpga_interconnect_pkg
        )
        port map (
            clock                    => simulator_clock
            ,spi_clock               => spi_clock
            ,spi_cs_in               => spi_cs_in
            ,spi_data_in             => spi_data_in
            ,spi_data_out            => spi_data_out
            ,bus_to_communications   => bus_to_communications
            ,bus_from_communications => bus_from_communications
        );
    end generate;

    u_dut : entity work.hw_test_core
    generic map (
        g_board_id            => 7
        ,g_clock_frequency_hz => 120_000_000
        ,g_dsp_pre_add_register => pre_add_register
        ,g_dsp_product_register => product_register
        ,g_mproc_program_ram_output_register => mproc_program_ram_register
        ,g_mproc_data_ram_output_register    => mproc_data_ram_register
        ,g_mproc_divider_shifter_stages      => mproc_divider_shifter_stages
        ,g_ram_output_register  => ram_output_register
        ,g_dsp_request_register => dsp_request_register
        ,g_divider_index_width       => divider_index_width
        ,g_divider_table_word_length => divider_table_word_length
        ,g_divider_table_radix       => divider_table_radix
        ,g_divider_x_frac_width      => divider_x_frac_width
        ,g_root_index_width       => root_index_width
        ,g_root_table_word_length => root_table_word_length
        ,g_root_table_radix       => root_table_radix
        ,g_root_x_frac_width      => root_x_frac_width
    )
    port map (
        clock      => simulator_clock
        ,reset     => reset
        ,bus_from_communications => bus_from_communications
        ,bus_to_communications   => bus_to_communications
        ,heartbeat => open
    );
------------------------------------------------------------------------
end vunit_simulation;
