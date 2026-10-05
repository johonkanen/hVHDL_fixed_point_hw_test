"""
make_peri.py - generate uart_test.peri.xml (the Interface Designer periphery)
for the Efinix Trion T120F324 development board with Efinity's python api

    source ~/efinity/2026.1/bin/setup.sh
    python3 make_peri.py [path to efinity_spi_comm.peri.xml]

Starts from the hardware proven periphery of the efinix_spi_communication
project (its spi pins, pll input and leds) :

  pll_input_clock  GPIOL_15, 30 MHz, PLL_BL0 external reference
  main_pll         -> main_clock at CORE_CLOCK_MHZ, locked -> pll_locked
  spi_clock        GPIOL_01  FT2232H channel A, mpsse spi
  spi_cs_in        GPIOL_00
  spi_data_in      GPIOL_08
  spi_data_out     GPIOL_09
  user_led[3:0]    GPIOT_RXP24 / RXN24 / RXP27 / RXN27

the board has no uart to the fpga, the pc reaches the registers over spi.
"""
import os
import sys

sys.path.append(os.environ["EFXPT_HOME"] + "/bin")

from api_service.design import DesignAPI

# the 30 MHz reference times 48 is a 1440 MHz vco, divided by 2 and the
# output divider : 12 gives 60 MHz
CORE_CLOCK_MHZ = 60.0
OUTPUT_DIVIDER = 12

here = os.path.dirname(os.path.abspath(__file__))
source = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/dev/efinix_spi_communication/efinity_spi_comm/efinity_spi_comm.peri.xml")
target = os.path.join(here, "uart_test.peri.xml")

with open(source) as f:
    text = f.read()
with open(target, "w") as f:
    f.write(text.replace('name="efinity_spi_comm"', 'name="uart_test"', 1))

design = DesignAPI(False)
design.load(target)

design.set_property("main_pll", {"LOCKED_PIN": "pll_locked", "CLKOUT0_DIV": str(OUTPUT_DIVIDER)}, block_type="PLL")
design.calc_pll_clock("main_pll")
frequency = float(design.get_property("main_pll", "CLKOUT0_FREQ", block_type="PLL")["CLKOUT0_FREQ"])
print("main_pll", design.get_property("main_pll", ["REFCLK_FREQ", "CLKOUT0_FREQ", "CLKOUT0_PIN", "LOCKED_PIN"], block_type="PLL"))
if abs(frequency - CORE_CLOCK_MHZ) > 1e-6:
    sys.exit(f"main_clock is {frequency} MHz, not {CORE_CLOCK_MHZ}")

for name in ("pll_input_clock", "spi_clock", "spi_cs_in", "spi_data_in", "spi_data_out"):
    print(name, design.get_resource(name))

design.check_design()
design.save()
