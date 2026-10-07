------------------------------------------------------------------------
-- mproc_test - uart register test for hVHDL_microprogam_processor's
-- microprogram_controller with the fixed point instruction
-- (instruction(fixed_mult_add)), 32 bit data at radix g_radix
--
-- registers from g_base_address :
--
--   +0 : start address in the program ram                        RW
--   +1 : write -> run the program from +0                         WO
--   +2 : 1 while the processor runs                               RO
--   +3 : ready pulses since the last run                          RO
--   +4 : clock edges from the run request to ready                RO
--   +5 : radix (g_radix)                                          RO
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
--        mpy_sub       2 <- 67 * 68 + not 69
--        neg_mpy_add   3 <- not 70 * 71 + 72
--        neg_mpy_sub   4 <- not 73 * 74 + not 75
--        a_add_b_mpy_c 5 <- (76 + 77) * 78
--        a_sub_b_mpy_c 6 <- (79 + not 80) * 81
--        lp_filter     7 <- (82 + not 83) * 84 + 83
--        acc 85, acc 86, get_acc_and_zero 8 <- 85 + 86 + 87
--   32 : set_rpt 99, then lp_filter 96 <- (97 + not 96) * 98 + 96 in a
--        loop closed by jump, 100 times
--
-- a product a * b + c * 2**radix is taken from bits radix + 31 downto
-- radix of the 64 bit result. not x is -x - 1. The sequencer reports
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
    );
    port (
        clock    : in std_logic
        ;reset   : in std_logic -- synchronous, active high
        ;bus_in  : in fpga_interconnect_record
        ;bus_out : out fpga_interconnect_record
    );
end entity mproc_test;

architecture rtl of mproc_test is

    use work.microprogram_processor_pkg.all;
    use work.microinstruction_pkg.all;
    use work.multi_port_ram_pkg.all;
    use work.instruction_pkg.all;

    constant word_length        : natural := 32;
    constant instruction_length : natural := 32;
    constant ram_size           : natural := 128;

    constant ref_subtype : subtype_ref_record :=
        create_ref_subtypes(readports => 3, datawidth => word_length, addresswidth => 10);
    constant instr_ref_subtype : subtype_ref_record :=
        create_ref_subtypes(readports => 1, datawidth => instruction_length, addresswidth => 10);

    constant program_data : work.dual_port_ram_pkg.ram_array(0 to ref_subtype.address_high)(ref_subtype.data'range) :=
        (others => (others => '0'));

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

        , others => op(nop));

    signal mproc_in  : microprogram_processor_in_record;
    signal mproc_out : microprogram_processor_out_record;

    signal mc_output   : ref_subtype.ram_write_in'subtype;
    signal mc_write_in : ref_subtype.ram_write_in'subtype := ref_subtype.ram_write_in;

    constant instruction_in_ref : instruction_in_record := (
        instr_ram_read_out => instr_ref_subtype.ram_read_out
        ,data_read_out     => ref_subtype.ram_read_out
        ,instr_pipeline    => (0 to 12 => op(nop))
    );
    constant instruction_out_ref : instruction_out_record := (
        data_read_in  => ref_subtype.ram_read_in
        ,ram_write_in => ref_subtype.ram_write_in
    );

    signal instr_in  : instruction_in_ref'subtype  := instruction_in_ref;
    signal instr_out : instruction_out_ref'subtype := instruction_out_ref;

    type shadow_array is array (0 to ram_size-1) of std_logic_vector(word_length-1 downto 0);
    signal shadow_ram   : shadow_array := (others => (others => '0'));
    signal shadow_q     : std_logic_vector(word_length-1 downto 0) := (others => '0');
    signal read_pending : boolean := false;

    signal start_address : std_logic_vector(31 downto 0) := (others => '0');
    signal running       : boolean := false;
    signal latency       : unsigned(31 downto 0) := (others => '0');
    signal ready_count   : unsigned(31 downto 0) := (others => '0');

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

            if write_is_requested_to_address(bus_in, g_base_address + 1) then
                calculate(mproc_in, to_integer(unsigned(start_address(9 downto 0))));
                running     <= true;
                latency     <= (others => '0');
                ready_count <= (others => '0');
            end if;

            if running then
                latency <= latency + 1;
            end if;
            if is_ready(mproc_out) then
                running     <= false;
                ready_count <= ready_count + 1;
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
                running <= false;
            end if;
        end if;
    end process registers;

    u_microprogram_controller : entity work.microprogram_controller
    generic map (g_program => test_program, g_data => program_data, g_data_bit_width => word_length)
    port map (
        clock            => clock
        ,mproc_in        => mproc_in
        ,mproc_out       => mproc_out
        ,mc_output       => mc_output
        ,mc_write_in     => mc_write_in
        ,instruction_in  => instr_in
        ,instruction_out => instr_out
    );

    u_fixed_mult_add : entity work.instruction(fixed_mult_add)
    generic map (radix => g_radix)
    port map (
        clock            => clock
        ,instruction_in  => instr_in
        ,instruction_out => instr_out
    );

end architecture rtl;
