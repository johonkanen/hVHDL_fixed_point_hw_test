------------------------------------------------------------------------
-- mproc_test - uart register test for hVHDL_microprogam_processor's
-- microprogram_core with the fixed point execution unit
-- (execution_unit(fixed_mult_add)), 32 bit data at radix g_radix
--
-- registers from g_base_address :
--
--   +0 : start address in the program ram                        RW
--   +1 : write -> run the program from +0                         WO
--   +2 : 1 while the processor runs                               RO
--   +3 : ready pulses since the last run                          RO
--   +4 : clock edges from the run request to ready                RO
--   +5 : radix (g_radix)                                          RO
--   +6 : bit 0 : run the boost converter model (program 128) in
--        the background                                           RW
--   +7 : clock edges from one background run to the next          RW
--   +8 : background runs since bit 0 of +6 was last set           RO
--   +9 : data word width (g_word_length)                          RO
--   +10: instruction width (g_instruction_length)                 RO
--   +11: data ram words, as far as the instructions' address fields
--        reach : 128 for 32 bit instructions, 256 for 36           RO
--   +12: result latency L : an instruction reads the result of one
--        at least L instructions before it                        RO
--   +13: jump delay slots S, 3 with the program ram's output
--        register, 2 without                                      RO
--   +14: the math unit's result latency, 0 without one            RO
--
-- the data ram, of g_word_length bits, from g_ram_base_address, bits
-- 31..0 :
--
--   +0..   : write -> data ram, read <- a copy of the data ram
--            kept from the processor's ram writes                 RW
--
-- and from g_ram_high_base_address the bits above 31, sign extended,
-- for a word length over 32 :
--
--   +0..   : read <- bits word length - 1 downto 32 of the word     RW
--            write -> the bits above 31 of the next word written
--            from g_ram_base_address (any address)
--
-- the programs are in the program ram, the operands and the results in
-- the data ram. Each is written as its instructions in order and laid out
-- for this instance by microprogram_assembler_pkg's schedule() and
-- repeat() with the execution unit's result latency L, so ready marks the
-- results as in the data ram. Run times, from the run request to ready :
--
--   0  : one of each fixed_mult_add command, operands from 64..87,
--        12 + S + L clocks
--        mpy_add       1 <- 64 * 65 + 66
--        mpy_sub       2 <- 67 * 68 - 69
--        neg_mpy_add   3 <- -70 * 71 + 72
--        neg_mpy_sub   4 <- -73 * 74 - 75
--        a_add_b_mpy_c 5 <- (76 + 77) * 78
--        a_sub_b_mpy_c 6 <- (79 - 80) * 81
--        lp_filter     7 <- (82 - 83) * 84 + 83
--        acc 85, acc 86, get_acc_and_zero 8 <- 85 + 86 + 87
--   32 : lp_filter 96 <- (97 - 96) * 98 + 96, repeated 100 times,
--        4 + S + 100 * L clocks
--   192: with 8 bit address fields or more, operands and results
--        above 127 : mpy_add 250 <- 200 * 201 + 202,
--        mpy_sub 251 <- 203 * 204 - 205, 4 + S + L clocks
--   224: with the math unit, 112 <- 110 / 111, then
--        113 <- 112 * 114 + 115 and 116 <- 113 / 111
--   288: with the math unit, 118 <- sqrt(117), then 119 <- 117 / 118
--   128: one time step of an averaged boost converter, 3 + S + 3 * L clocks (the
--        ac_in_ac_out_lab_power_supply test_processor v3 model), the
--        inductor current i and capacitor voltage u from the input
--        voltage, the duty d (as the switch's 1 - D) and the load
--        current, Euler steps with h/L and h/C :
--          vL <- -d * u + vin           ic <- d * i - load
--          vL <- -r * i + vL            u  <- ic * h/C + u
--          i  <- vL * h/L + i
--        in the data ram
--          100 vin   101 d   102 load   103 r   104 h/L   105 h/C
--          106 i     107 u   108 vL     109 ic
--        and starts at vin 20, d 0.8, load 0, r 0.8, h/L = h/C = 0.7/3,
--        i 0, u 12. With the background runs on, the model steps every
--        +7 clocks while the processor is idle and vin, d, load and r
--        can be written while it runs. Turn it off before running a
--        program from +1.
--
-- fixed_mult_add runs on fixed_dsp : the sums, differences and -x wrap
-- to 32 bits in its pre-adder, a product a * b +- c * 2**radix is taken
-- from bits radix + 31 downto radix of the 64 bit result. The sequencer reports
-- ready when it reads program_end, a few clocks before the last result is
-- in the data ram. The three instructions after a jump are already
-- fetched when it is taken and run every round, a program_end there
-- would end the loop.
------------------------------------------------------------------------
library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;

entity mproc_test is
    generic (
        g_base_address      : natural
        ;g_ram_base_address : natural
        ;g_ram_high_base_address : natural
        ;g_word_length        : natural := 32 -- 32..64
        ;g_instruction_length : natural := 32 -- 32 and up, see generic_microinstruction_pkg
        ;g_radix            : natural := 20
        ;g_pre_add_register : boolean := false -- fixed_dsp's
        ;g_product_register : boolean := false -- fixed_dsp's
        -- the processor's rams' output registers, microprogram_core's
        ;g_program_ram_output_register : boolean := true
        ;g_data_ram_output_register    : boolean := true
        -- a fixed_math unit (division, square root) beside fixed_mult_add, and its
        -- divider's shifter stages
        ;g_math_unit : boolean := false
        ;g_divider_shifter_stages : positive := 2
    );
    port (
        clock    : in std_logic
        ;reset   : in std_logic -- synchronous, active high
        ;bus_in  : in fpga_interconnect_record
        ;bus_out : out fpga_interconnect_record
    );
end entity mproc_test;

architecture rtl of mproc_test is

    use work.microprogram_interface_pkg.all;
    use work.microinstruction_pkg.all;
    use work.multi_port_ram_pkg.all;
    use work.execution_unit_pkg.all;
    use work.microprogram_assembler_pkg.all;
    use work.boost_converter_pkg.all;

    constant word_length        : natural := g_word_length;
    constant instruction_length : natural := g_instruction_length;
    -- the data ram as far as the address fields reach
    constant ram_size           : natural := 2**address_bits(instruction_length);

    constant ref_subtype : subtype_ref_record :=
        create_ref_subtypes(readports => 3, datawidth => word_length, addresswidth => 10);
    constant instr_ref_subtype : subtype_ref_record :=
        create_ref_subtypes(readports => 1, datawidth => instruction_length, addresswidth => 10);

    -- the boost converter model of the processor's boost_converter_pkg,
    -- its data from 100, its program at 128
    constant boost : boost_converter_map := boost_converter_at(100);
    constant boost_converter : natural := 128;

    -- the math unit writes later than the multiply-adds, the instruction
    -- pipeline reaches its result stage
    function choose_math_latency return natural is
    begin
        if g_math_unit then
            return fixed_math_result_latency(g_pre_add_register, g_product_register, g_data_ram_output_register,
                g_divider_shifter_stages);
        end if;
        return 0;
    end choose_math_latency;
    constant math_latency  : natural := choose_math_latency;
    constant pipeline_high : natural := maximum(12, math_latency);

    -- the programs, laid out for this instance's configuration
    constant config : processor_config := (
        instruction_width => instruction_length
        ,data_width       => word_length
        ,radix            => g_radix
        ,result_latency   => fixed_point_result_latency(g_pre_add_register, g_product_register, g_data_ram_output_register)
        ,delay_slots      => jump_delay_slots(g_program_ram_output_register)
        ,math_latency     => math_latency);

    constant one_of_each : microprogram := (
         mi(mpy_add          , 1 , 64 , 65 , 66)
        ,mi(mpy_sub          , 2 , 67 , 68 , 69)
        ,mi(neg_mpy_add      , 3 , 70 , 71 , 72)
        ,mi(neg_mpy_sub      , 4 , 73 , 74 , 75)
        ,mi(a_add_b_mpy_c    , 5 , 76 , 77 , 78)
        ,mi(a_sub_b_mpy_c    , 6 , 79 , 80 , 81)
        ,mi(lp_filter        , 7 , 82 , 83 , 84)
        ,mi(acc              , 0 , 0  , 0  , 85)
        ,mi(acc              , 0 , 0  , 0  , 86)
        ,mi(get_acc_and_zero , 8 , 0  , 0  , 87));

    constant low_pass_filter : microprogram := (0 => mi(lp_filter, 96, 97, 96, 98));

    constant divisions : microprogram := (
         mi_div(112, 110, 111)
        ,mi(mpy_add, 113, 112, 114, 115)
        ,mi_div(116, 113, 111));

    constant square_roots : microprogram := (
         mi_sqrt(118, 117)
        ,mi_div(119, 117, 118));

    constant high_addresses : microprogram := (
         mi(mpy_add , 250 , 200 , 201 , 202)
        ,mi(mpy_sub , 251 , 203 , 204 , 205));

    function make_program return microprogram is
        variable retval : microprogram(0 to instr_ref_subtype.address_high) := empty_program(instr_ref_subtype.address_high + 1);
    begin
        retval := place(retval, 0,   schedule(config, one_of_each) & mi(program_end));
        retval := place(retval, 32,  repeat(config, 100, low_pass_filter) & mi(program_end));
        retval := place(retval, boost_converter, schedule(config, boost_converter_step(boost)) & mi(program_end));
        if address_bits(instruction_length) >= 8 then
            retval := place(retval, 192, schedule(config, high_addresses) & mi(program_end));
        end if;
        if g_math_unit then
            retval := place(retval, 224, schedule(config, divisions) & mi(program_end));
            retval := place(retval, 288, schedule(config, square_roots) & mi(program_end));
        end if;
        return retval;
    end make_program;

    constant program_data : work.dual_port_ram_pkg.ram_array(0 to ref_subtype.address_high)(ref_subtype.data'range)
        := encode_data(boost_converter_data(boost, boost_converter_example), config, ref_subtype.address_high + 1);

    constant test_program : work.dual_port_ram_pkg.ram_array(0 to instr_ref_subtype.address_high)(instr_ref_subtype.data'range)
        := encode(make_program, instruction_length);

    signal mproc_in  : microprogram_processor_in_record;
    signal mproc_out : microprogram_processor_out_record;

    signal mc_output   : ref_subtype.ram_write_in'subtype;
    signal mc_write_in : ref_subtype.ram_write_in'subtype := ref_subtype.ram_write_in;

    constant unit_in_ref : execution_unit_in_record := (
        instr_ram_read_out => instr_ref_subtype.ram_read_out
        ,data_read_out     => ref_subtype.ram_read_out
        ,instr_pipeline    => (0 to pipeline_high => encode(mi(nop), instruction_length))
    );
    constant unit_out_ref : execution_unit_out_record := (
        data_read_in  => ref_subtype.ram_read_in
        ,ram_write_in => ref_subtype.ram_write_in
    );

    signal unit_in  : unit_in_ref'subtype  := unit_in_ref;
    signal unit_out : unit_out_ref'subtype := unit_out_ref;
    signal mult_add_out : unit_out_ref'subtype := unit_out_ref;
    signal math_out     : unit_out_ref'subtype := unit_out_ref;

    type shadow_array is array (0 to ram_size-1) of std_logic_vector(word_length-1 downto 0);

    function initial_shadow return shadow_array is
        variable retval : shadow_array;
    begin
        for k in retval'range loop
            retval(k) := program_data(k);
        end loop;
        return retval;
    end initial_shadow;

    signal shadow_ram   : shadow_array := initial_shadow;
    signal shadow_q     : std_logic_vector(word_length-1 downto 0) := (others => '0');
    signal read_pending : boolean := false;
    signal read_high    : boolean := false;
    signal high_bits    : std_logic_vector(31 downto 0) := (others => '0');

    -- a word from the bus' low word and the held high bits
    function to_word (low, high : std_logic_vector(31 downto 0)) return std_logic_vector is
        variable retval : std_logic_vector(63 downto 0);
    begin
        retval := high & low;
        return retval(word_length-1 downto 0);
    end to_word;

    -- bits 31..0 or the bits above, sign extended, of a word
    function word_part (word : std_logic_vector; high : boolean) return std_logic_vector is
        constant extended : signed(63 downto 0) := resize(signed(word), 64);
    begin
        if high then
            return std_logic_vector(extended(63 downto 32));
        end if;
        return std_logic_vector(extended(31 downto 0));
    end word_part;

    signal start_address : std_logic_vector(31 downto 0) := (others => '0');
    signal running       : boolean := false;
    signal latency       : unsigned(31 downto 0) := (others => '0');
    signal ready_count   : unsigned(31 downto 0) := (others => '0');

    signal background        : std_logic_vector(31 downto 0) := (others => '0');
    signal background_period : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(1000, 32));
    signal background_timer  : unsigned(31 downto 0) := (others => '0');
    signal background_runs   : unsigned(31 downto 0) := (others => '0');
    signal in_background     : boolean := false;

begin

    registers : process (clock) is
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            init_mproc(mproc_in);
            init_mp_write(mc_write_in);

            connect_data_to_address(bus_in, bus_out, g_base_address, start_address);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 2, std_logic_vector(to_unsigned(boolean'pos(mproc_out.is_busy), 32)));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 3, std_logic_vector(ready_count));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 4, std_logic_vector(latency));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 5, std_logic_vector(to_unsigned(g_radix, 32)));
            connect_data_to_address(bus_in, bus_out, g_base_address + 6, background);
            connect_data_to_address(bus_in, bus_out, g_base_address + 7, background_period);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 8, std_logic_vector(background_runs));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 9, std_logic_vector(to_unsigned(word_length, 32)));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 10, std_logic_vector(to_unsigned(instruction_length, 32)));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 11, std_logic_vector(to_unsigned(ram_size, 32)));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 12, std_logic_vector(to_unsigned(config.result_latency, 32)));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 13, std_logic_vector(to_unsigned(config.delay_slots, 32)));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 14, std_logic_vector(to_unsigned(config.math_latency, 32)));

            if write_is_requested_to_address(bus_in, g_base_address + 1) then
                calculate(mproc_in, to_integer(unsigned(start_address(9 downto 0))));
                running     <= true;
                latency     <= (others => '0');
                ready_count <= (others => '0');
            end if;

            if running then
                latency <= latency + 1;
            end if;
            if is_ready(mproc_out) and running then
                running     <= false;
                ready_count <= ready_count + 1;
            end if;

            -- the boost converter model every background_period clocks,
            -- a run that falls on a busy processor waits for it
            if write_is_requested_to_address(bus_in, g_base_address + 6)
                and get_slv_data(bus_in)(0) = '1'
            then
                background_runs <= (others => '0');
            end if;

            background_timer <= background_timer + 1;
            if background_timer + 1 >= unsigned(background_period) then
                background_timer <= unsigned(background_period);
            end if;

            if background(0) = '1'
                and background_timer + 1 >= unsigned(background_period)
                and not mproc_out.is_busy and not in_background and not running
                and not write_is_requested_to_address(bus_in, g_base_address + 1)
            then
                calculate(mproc_in, boost_converter);
                in_background    <= true;
                background_timer <= (others => '0');
            end if;

            if is_ready(mproc_out) and in_background then
                in_background   <= false;
                background_runs <= background_runs + 1;
            end if;

            -- the data ram : writes go to the controller, reads come from
            -- the copy one clock later
            if write_is_requested_to_address_range(bus_in, g_ram_base_address, g_ram_base_address + ram_size) then
                write_data_to_ram(mc_write_in, get_address(bus_in) - g_ram_base_address, to_word(get_slv_data(bus_in), high_bits));
            end if;
            if write_is_requested_to_address_range(bus_in, g_ram_high_base_address, g_ram_high_base_address + ram_size) then
                high_bits <= get_slv_data(bus_in);
            end if;

            if mc_output.write_requested = '1' then
                shadow_ram(to_integer(mc_output.address) mod ram_size) <= mc_output.data;
            end if;
            shadow_q     <= shadow_ram(get_address(bus_in) mod ram_size);
            read_pending <= data_is_requested_from_address_range(bus_in, g_ram_base_address, g_ram_base_address + ram_size)
                or data_is_requested_from_address_range(bus_in, g_ram_high_base_address, g_ram_high_base_address + ram_size);
            read_high <= data_is_requested_from_address_range(bus_in, g_ram_high_base_address, g_ram_high_base_address + ram_size);
            if read_pending then
                write_data_to_address(bus_out, 0, word_part(shadow_q, read_high));
            end if;

            if reset = '1' then
                running       <= false;
                in_background <= false;
                background    <= (others => '0');
            end if;
        end if;
    end process registers;

    u_microprogram_core : entity work.microprogram_core
    generic map (g_program => test_program, g_data => program_data
        ,g_program_ram_output_register => g_program_ram_output_register
        ,g_data_ram_output_register    => g_data_ram_output_register)
    port map (
        clock            => clock
        ,mproc_in        => mproc_in
        ,mproc_out       => mproc_out
        ,mc_output       => mc_output
        ,mc_write_in     => mc_write_in
        ,to_unit         => unit_in
        ,from_unit       => unit_out
    );

    u_fixed_mult_add : entity work.execution_unit(fixed_mult_add)
    generic map (g_radix => g_radix, g_pre_add_register => g_pre_add_register, g_product_register => g_product_register
        ,g_data_ram_output_register => g_data_ram_output_register)
    port map (
        clock            => clock
        ,unit_in         => unit_in
        ,unit_out        => mult_add_out
    );

    no_math : if not g_math_unit generate
        unit_out <= mult_add_out;
    end generate;

    math : if g_math_unit generate
        u_fixed_math : entity work.execution_unit(fixed_math)
        generic map (g_radix => g_radix, g_pre_add_register => g_pre_add_register, g_product_register => g_product_register
            ,g_data_ram_output_register => g_data_ram_output_register, g_divider_shifter_stages => g_divider_shifter_stages)
        port map (
            clock            => clock
            ,unit_in         => unit_in
            ,unit_out        => math_out
        );

        unit_out <= merge_units(mult_add_out, math_out);
    end generate;

end architecture rtl;
