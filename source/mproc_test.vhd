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
--
-- the data ram, 128 words from g_ram_base_address :
--
--   +0..+127 : write -> data ram, read <- a copy of the data ram
--              kept from the controller's ram writes              RW
--
-- the programs are in the program ram, the operands and the results in
-- the data ram :
--
--   0  : one of each fixed_mult_add command, operands from 64..87
--        mpy_add       1 <- 64 * 65 + 66
--        mpy_sub       2 <- 67 * 68 - 69
--        neg_mpy_add   3 <- -70 * 71 + 72
--        neg_mpy_sub   4 <- -73 * 74 - 75
--        a_add_b_mpy_c 5 <- (76 + 77) * 78
--        a_sub_b_mpy_c 6 <- (79 - 80) * 81
--        lp_filter     7 <- (82 - 83) * 84 + 83
--        acc 85, acc 86, get_acc_and_zero 8 <- 85 + 86 + 87
--   32 : set_rpt 99, then lp_filter 96 <- (97 - 96) * 98 + 96 in a
--        loop closed by jump, 100 times
--   128: one time step of an averaged boost converter (the
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
        ;g_radix            : natural := 20
        ;g_pre_add_register : boolean := false -- fixed_dsp's
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

    constant word_length        : natural := 32;
    constant instruction_length : natural := 32;
    constant ram_size           : natural := 128;

    constant ref_subtype : subtype_ref_record :=
        create_ref_subtypes(readports => 3, datawidth => word_length, addresswidth => 10);
    constant instr_ref_subtype : subtype_ref_record :=
        create_ref_subtypes(readports => 1, datawidth => instruction_length, addresswidth => 10);

    -- the boost converter model's data
    constant vin      : natural := 100;
    constant duty     : natural := 101;
    constant load     : natural := 102;
    constant r        : natural := 103;
    constant i_gain   : natural := 104;
    constant u_gain   : natural := 105;
    constant i        : natural := 106;
    constant u        : natural := 107;
    constant vl       : natural := 108;
    constant ic       : natural := 109;
    constant boost_converter : natural := 128;

    function to_fixed (x : real) return std_logic_vector is
    begin
        return std_logic_vector(to_signed(integer(x * 2.0**g_radix), word_length));
    end to_fixed;

    constant program_data : work.dual_port_ram_pkg.ram_array(0 to ref_subtype.address_high)(ref_subtype.data'range) := (
        vin      => to_fixed(20.0)
        ,duty    => to_fixed(0.8)
        ,load    => to_fixed(0.0)
        ,r       => to_fixed(0.8)
        ,i_gain  => to_fixed(0.7 / 3.0)
        ,u_gain  => to_fixed(0.7 / 3.0)
        ,i       => to_fixed(0.0)
        ,u       => to_fixed(12.0)
        ,others  => (others => '0'));

    -- results depend on operands written at least 16 instructions before
    constant test_program : work.dual_port_ram_pkg.ram_array(0 to instr_ref_subtype.address_high)(instr_ref_subtype.data'range) := (
        0   => op(mpy_add          , 1 , 64 , 65 , 66)
        , 1 => op(mpy_sub          , 2 , 67 , 68 , 69)
        , 2 => op(neg_mpy_add      , 3 , 70 , 71 , 72)
        , 3 => op(neg_mpy_sub      , 4 , 73 , 74 , 75)
        , 4 => op(a_add_b_mpy_c    , 5 , 76 , 77 , 78)
        , 5 => op(a_sub_b_mpy_c    , 6 , 79 , 80 , 81)
        , 6 => op(lp_filter        , 7 , 82 , 83 , 84)
        , 7 => op(acc              , 0 , 0  , 0  , 85)
        , 8 => op(acc              , 0 , 0  , 0  , 86)
        , 9 => op(get_acc_and_zero , 8 , 0  , 0  , 87)
        , 10 => op(program_end)

        , 32 => op(set_rpt   , 99)
        , 33 => op(lp_filter , 96 , 97 , 96 , 98)
        , 49 => op(jump      , 33)
        , 53 => op(program_end)

        , boost_converter      => op(neg_mpy_add , vl , duty   , u      , vin)
        , boost_converter + 1  => op(mpy_sub     , ic , duty   , i      , load)
        , boost_converter + 13 => op(neg_mpy_add , vl , r      , i      , vl)
        , boost_converter + 14 => op(mpy_add     , u  , ic     , u_gain , u)
        , boost_converter + 28 => op(mpy_add     , i  , vl     , i_gain , i)
        , boost_converter + 30 => op(program_end)

        , others => op(nop));

    signal mproc_in  : microprogram_processor_in_record;
    signal mproc_out : microprogram_processor_out_record;

    signal mc_output   : ref_subtype.ram_write_in'subtype;
    signal mc_write_in : ref_subtype.ram_write_in'subtype := ref_subtype.ram_write_in;

    constant unit_in_ref : execution_unit_in_record := (
        instr_ram_read_out => instr_ref_subtype.ram_read_out
        ,data_read_out     => ref_subtype.ram_read_out
        ,instr_pipeline    => (0 to 12 => op(nop))
    );
    constant unit_out_ref : execution_unit_out_record := (
        data_read_in  => ref_subtype.ram_read_in
        ,ram_write_in => ref_subtype.ram_write_in
    );

    signal unit_in  : unit_in_ref'subtype  := unit_in_ref;
    signal unit_out : unit_out_ref'subtype := unit_out_ref;

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
                write_data_to_ram(mc_write_in, get_address(bus_in) - g_ram_base_address, get_slv_data(bus_in));
            end if;

            if mc_output.write_requested = '1' then
                shadow_ram(to_integer(mc_output.address) mod ram_size) <= mc_output.data;
            end if;
            shadow_q     <= shadow_ram(get_address(bus_in) mod ram_size);
            read_pending <= data_is_requested_from_address_range(bus_in, g_ram_base_address, g_ram_base_address + ram_size);
            if read_pending then
                write_data_to_address(bus_out, 0, shadow_q);
            end if;

            if reset = '1' then
                running       <= false;
                in_background <= false;
                background    <= (others => '0');
            end if;
        end if;
    end process registers;

    u_microprogram_core : entity work.microprogram_core
    generic map (g_program => test_program, g_data => program_data, g_data_bit_width => word_length)
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
    generic map (g_radix => g_radix, g_pre_add_register => g_pre_add_register)
    port map (
        clock            => clock
        ,unit_in         => unit_in
        ,unit_out        => unit_out
    );

end architecture rtl;
