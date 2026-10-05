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

    signal reset_meta   : std_logic := '1';
    signal system_reset : std_logic := '1';

    signal bus_to_communications   : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_communications : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_top            : fpga_interconnect_record := init_fpga_interconnect;

    signal loopback_register : std_logic_vector(31 downto 0) := (others => '0');
    signal read_counter      : unsigned(31 downto 0) := (others => '0');
    signal clock_counter     : unsigned(31 downto 0) := (others => '0');

    constant first_bank_address : natural := 16;
    constant bank_size          : natural := 16;
    type register_array is array (0 to bank_size-1) of std_logic_vector(31 downto 0);
    signal register_bank : register_array := (others => (others => '0'));

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
            connect_read_only_data_to_address(bus_from_communications, bus_from_top, 2, work.git_hash_pkg.git_hash);
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

            bus_to_communications <= bus_from_top;

            if system_reset = '1' then
                loopback_register     <= (others => '0');
                read_counter          <= (others => '0');
                register_bank         <= (others => (others => '0'));
                bus_to_communications <= init_fpga_interconnect;
            end if;
        end if;
    end process test_registers;

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
