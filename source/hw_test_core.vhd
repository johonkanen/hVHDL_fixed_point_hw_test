------------------------------------------------------------------------
-- hw_test_core - board independent register test
--
-- the registers on fpga_interconnect (32 bit data, 16 bit address) ; the
-- board top connects the buses to its link to the pc, fpga_communication's
-- uart (fpga_communications) or efinix_spi_communication's spi
-- (fpga_spi_communications) :
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
--   45 : 1 when g_dsp_pre_add_register is set (latency 3)       RO
--   46 : 1 when g_ram_output_register is set                    RO
--   47 : 1 when g_dsp_request_register is set                   RO
--
-- sine_calculator, reciprocal_calculator and sqrt_calculator, each with its own fixed_dsp
-- (same width and pre-add option), tested through lut_sweep :
--
--   48..57 : sine_calculator, 16 bit angle (fraction of a turn) ->
--            16 bit signed sine
--   64..73 : reciprocal_calculator, 16 bit x_frac (x = 0.5 + x_frac/2**17)
--            -> 16 bit unsigned 1/x
--   80..89 : sqrt_calculator, 16 bit x_frac (x = 0.5 + x_frac/2**17)
--            -> 16 bit unsigned sqrt(x)
--
--   base +0 input  +1 single request  +2 result  +3 latency
--        +4 sweep N  +5 sweep start  +6 s1  +7 s2  +8 readies  +9 mode
--   (see lut_sweep.vhd)
--
-- lut_divider, 32 bit numerator / denominator, quotient radix 16, own
-- fixed_dsps with the same pre-add option :
--
--   96..111 : see divider_sweep.vhd, the reciprocal table from the
--             g_divider_* generics
--
-- full_range_sqrt, 32 bit radicand and root, radix 16, own fixed_dsps with
-- the same pre-add option :
--
--   112..124 : see sqrt_sweep.vhd, the sqrt table from the g_root_*
--              generics
--
--   125 : 1 when g_dsp_product_register is set, every fixed_dsp one clock
--         longer                                                RO
--
-- hVHDL_microprogam_processor's microprogram_core with
-- execution_unit(fixed_mult_add) on a fixed_dsp with the same pre-add option,
-- 32 bit data at radix 20 :
--
--   128..142 : see mproc_test.vhd
--   256..383 : its 128 word data ram, 384..511 the bits above 31 (none)
--
-- a second one with 36 bit data and instructions (8 bit address fields)
-- at radix 24, with a math unit (division, square root) :
--
--   144..158  : see mproc_test.vhd
--   512..767  : its 256 word data ram's bits 31..0, 768..1023 bits 35..32
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

    use work.fpga_interconnect_pkg.all;

entity hw_test_core is
    generic (
        g_board_id            : natural := 0
        ;g_clock_frequency_hz : natural := 120_000_000
        ;g_dsp_word_length    : natural := 32 -- 2..32
        ;g_dsp_pre_add_register : boolean := false -- fixed_dsp g_pre_add_register
        -- fixed_dsp g_product_register in every fixed_dsp, one clock more
        -- in each
        ;g_dsp_product_register : boolean := false
        -- the microprogram processors' program and data rams' output
        -- registers : without them a jump has 2 delay slots instead of 3
        -- and a result is readable a clock sooner
        ;g_mproc_program_ram_output_register : boolean := true
        ;g_mproc_data_ram_output_register    : boolean := true
        -- the 36 bit processor's math unit : its lut_divider's shifter
        -- stages, 2 clocks more each
        ;g_mproc_divider_shifter_stages : positive := 2
        -- dual_port_ram's output register in the lookup tables, off takes a
        -- clock off every calculator, the divider and the square root
        ;g_ram_output_register  : boolean := true
        -- the register on the requests to the calculators', the divider's
        -- and the square root's fixed_dsps, off takes a clock off each
        ;g_dsp_request_register : boolean := true
        -- lut_divider's reciprocal table, the default fills the ram blocks
        -- the 256 x 16 bit table already takes on all three boards
        ;g_divider_index_width       : natural := 9
        ;g_divider_table_word_length : natural := 18
        ;g_divider_table_radix       : natural := 16
        ;g_divider_x_frac_width      : natural := 18
        -- full_range_sqrt's sqrt table, the default fills the ram blocks the
        -- 256 x 16 bit table already takes on all three boards
        ;g_root_index_width       : natural := 9
        ;g_root_table_word_length : natural := 18
        ;g_root_table_radix       : natural := 17
        ;g_root_x_frac_width      : natural := 18
    );
    port (
        clock      : in std_logic
        ;reset     : in std_logic
        -- from and to the link's fpga_communications / fpga_spi_communications
        ;bus_from_communications : in fpga_interconnect_record
        ;bus_to_communications   : out fpga_interconnect_record := init_fpga_interconnect
        ;heartbeat : out std_logic -- toggles at about 1 Hz
    );
end entity hw_test_core;

architecture rtl of hw_test_core is

    use work.fixed_dsp_pkg.all;
    use work.git_hash_pkg.all;
    use work.sine_calculator_pkg.all;
    use work.reciprocal_calculator_pkg.all;
    use work.sqrt_calculator_pkg.all;

    signal reset_meta   : std_logic := '1';
    signal system_reset : std_logic := '1';

    signal bus_from_top            : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_dsp            : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_sine           : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_reciprocal     : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_sqrt           : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_divider        : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_root           : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_mproc          : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_mproc36        : fpga_interconnect_record := init_fpga_interconnect;

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
    signal sine_in      : sine_calculator_in_record;
    signal sine_out     : sine_calculator_out_record;
    signal sine_dsp_in  : dsp_in_subtype;
    signal sine_dsp_out : dsp_out_subtype;

    signal sine_request_value : unsigned(15 downto 0);
    signal sine_request       : std_logic;

    signal reciprocal_in       : reciprocal_calculator_in_record(x_frac(15 downto 0));
    signal reciprocal_out      : reciprocal_calculator_out_record(y(15 downto 0));
    signal reciprocal_dsp_in   : dsp_in_subtype;
    signal reciprocal_dsp_out  : dsp_out_subtype;
    signal reciprocal_request_value : unsigned(15 downto 0);
    signal reciprocal_request  : std_logic;

    signal sqrt_in            : sqrt_calculator_in_record(x_frac(15 downto 0));
    signal sqrt_out           : sqrt_calculator_out_record(y(15 downto 0));
    signal sqrt_dsp_in        : dsp_in_subtype;
    signal sqrt_dsp_out       : dsp_out_subtype;
    signal sqrt_request_value : unsigned(15 downto 0);
    signal sqrt_request       : std_logic;
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
            connect_read_only_data_to_address(bus_from_communications, bus_from_top, 125, std_logic_vector(to_unsigned(boolean'pos(g_dsp_product_register), 32)));

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

            bus_to_communications <= bus_from_top and bus_from_dsp and bus_from_sine and bus_from_reciprocal and bus_from_sqrt and bus_from_divider and bus_from_root and bus_from_mproc and bus_from_mproc36;

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
            connect_read_only_data_to_address(bus_from_communications, bus_from_dsp, 45, std_logic_vector(to_unsigned(boolean'pos(g_dsp_pre_add_register), 32)));
            connect_read_only_data_to_address(bus_from_communications, bus_from_dsp, 46, std_logic_vector(to_unsigned(boolean'pos(g_ram_output_register), 32)));
            connect_read_only_data_to_address(bus_from_communications, bus_from_dsp, 47, std_logic_vector(to_unsigned(boolean'pos(g_dsp_request_register), 32)));

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
    generic map (g_pre_add_register => g_dsp_pre_add_register, g_product_register => g_dsp_product_register)
    port map (
        clock          => clock
        ,fixed_dsp_in  => dsp_in
        ,fixed_dsp_out => dsp_out
    );

------------------------------------------------------------------------
    u_sine_sweep : entity work.lut_sweep
    generic map (g_base_address => 48, g_signed_result => true)
    port map (
        clock           => clock
        ,reset          => system_reset
        ,bus_in         => bus_from_communications
        ,bus_out        => bus_from_sine
        ,request_value  => sine_request_value
        ,request_with_1 => sine_request
        ,result         => std_logic_vector(sine_out.sine)
        ,ready_with_1   => sine_out.ready_with_1
    );

    sine_in <= (angle => sine_request_value, request_with_1 => sine_request);

    u_sine_calculator : entity work.sine_calculator
    generic map (
        g_ram_output_register   => g_ram_output_register
        ,g_dsp_request_register => g_dsp_request_register)
    port map (
        clock                => clock
        ,sine_calculator_in  => sine_in
        ,sine_calculator_out => sine_out
        ,fixed_dsp_in        => sine_dsp_in
        ,fixed_dsp_out       => sine_dsp_out
    );

    u_sine_dsp : entity work.fixed_dsp(rtl)
    generic map (g_pre_add_register => g_dsp_pre_add_register, g_product_register => g_dsp_product_register)
    port map (
        clock          => clock
        ,fixed_dsp_in  => sine_dsp_in
        ,fixed_dsp_out => sine_dsp_out
    );

------------------------------------------------------------------------
    u_reciprocal_sweep : entity work.lut_sweep
    generic map (g_base_address => 64, g_signed_result => false)
    port map (
        clock           => clock
        ,reset          => system_reset
        ,bus_in         => bus_from_communications
        ,bus_out        => bus_from_reciprocal
        ,request_value  => reciprocal_request_value
        ,request_with_1 => reciprocal_request
        ,result         => std_logic_vector(reciprocal_out.y)
        ,ready_with_1   => reciprocal_out.ready_with_1
    );

    reciprocal_in <= (x_frac => reciprocal_request_value, request_with_1 => reciprocal_request);

    u_reciprocal_calculator : entity work.reciprocal_calculator
    generic map (
        g_ram_output_register   => g_ram_output_register
        ,g_dsp_request_register => g_dsp_request_register)
    port map (
        clock                      => clock
        ,reciprocal_calculator_in  => reciprocal_in
        ,reciprocal_calculator_out => reciprocal_out
        ,fixed_dsp_in              => reciprocal_dsp_in
        ,fixed_dsp_out             => reciprocal_dsp_out
    );

    u_reciprocal_dsp : entity work.fixed_dsp(rtl)
    generic map (g_pre_add_register => g_dsp_pre_add_register, g_product_register => g_dsp_product_register)
    port map (
        clock          => clock
        ,fixed_dsp_in  => reciprocal_dsp_in
        ,fixed_dsp_out => reciprocal_dsp_out
    );

------------------------------------------------------------------------
    u_sqrt_sweep : entity work.lut_sweep
    generic map (g_base_address => 80, g_signed_result => false)
    port map (
        clock           => clock
        ,reset          => system_reset
        ,bus_in         => bus_from_communications
        ,bus_out        => bus_from_sqrt
        ,request_value  => sqrt_request_value
        ,request_with_1 => sqrt_request
        ,result         => std_logic_vector(sqrt_out.y)
        ,ready_with_1   => sqrt_out.ready_with_1
    );

    sqrt_in <= (x_frac => sqrt_request_value, request_with_1 => sqrt_request);

    u_sqrt_calculator : entity work.sqrt_calculator
    generic map (
        g_ram_output_register   => g_ram_output_register
        ,g_dsp_request_register => g_dsp_request_register)
    port map (
        clock                => clock
        ,sqrt_calculator_in  => sqrt_in
        ,sqrt_calculator_out => sqrt_out
        ,fixed_dsp_in        => sqrt_dsp_in
        ,fixed_dsp_out       => sqrt_dsp_out
    );

    u_sqrt_dsp : entity work.fixed_dsp(rtl)
    generic map (g_pre_add_register => g_dsp_pre_add_register, g_product_register => g_dsp_product_register)
    port map (
        clock          => clock
        ,fixed_dsp_in  => sqrt_dsp_in
        ,fixed_dsp_out => sqrt_dsp_out
    );

------------------------------------------------------------------------
    u_divider_sweep : entity work.divider_sweep
    generic map (
        g_base_address      => 96
        ,g_quotient_radix   => 16
        ,g_index_width       => g_divider_index_width
        ,g_table_word_length => g_divider_table_word_length
        ,g_table_radix       => g_divider_table_radix
        ,g_x_frac_width      => g_divider_x_frac_width
        ,g_pre_add_register => g_dsp_pre_add_register
        ,g_product_register => g_dsp_product_register
        ,g_ram_output_register => g_ram_output_register
        ,g_dsp_request_register => g_dsp_request_register
    )
    port map (
        clock    => clock
        ,reset   => system_reset
        ,bus_in  => bus_from_communications
        ,bus_out => bus_from_divider
    );

------------------------------------------------------------------------
    u_root_sweep : entity work.sqrt_sweep
    generic map (
        g_base_address      => 112
        ,g_radix            => 16
        ,g_index_width       => g_root_index_width
        ,g_table_word_length => g_root_table_word_length
        ,g_table_radix       => g_root_table_radix
        ,g_x_frac_width      => g_root_x_frac_width
        ,g_pre_add_register => g_dsp_pre_add_register
        ,g_product_register => g_dsp_product_register
        ,g_ram_output_register => g_ram_output_register
        ,g_dsp_request_register => g_dsp_request_register
    )
    port map (
        clock    => clock
        ,reset   => system_reset
        ,bus_in  => bus_from_communications
        ,bus_out => bus_from_root
    );

------------------------------------------------------------------------
    u_mproc_test : entity work.mproc_test
    generic map (g_base_address => 128, g_ram_base_address => 256, g_ram_high_base_address => 384
        ,g_pre_add_register => g_dsp_pre_add_register
        ,g_product_register => g_dsp_product_register
        ,g_program_ram_output_register => g_mproc_program_ram_output_register
        ,g_data_ram_output_register    => g_mproc_data_ram_output_register)
    port map (
        clock    => clock
        ,reset   => system_reset
        ,bus_in  => bus_from_communications
        ,bus_out => bus_from_mproc
    );

    u_mproc36_test : entity work.mproc_test
    generic map (g_base_address => 144, g_ram_base_address => 512, g_ram_high_base_address => 768
        ,g_word_length => 36, g_instruction_length => 36, g_radix => 24
        ,g_math_unit => true, g_divider_shifter_stages => g_mproc_divider_shifter_stages
        ,g_pre_add_register => g_dsp_pre_add_register
        ,g_product_register => g_dsp_product_register
        ,g_program_ram_output_register => g_mproc_program_ram_output_register
        ,g_data_ram_output_register    => g_mproc_data_ram_output_register)
    port map (
        clock    => clock
        ,reset   => system_reset
        ,bus_in  => bus_from_communications
        ,bus_out => bus_from_mproc36
    );

end architecture rtl;
