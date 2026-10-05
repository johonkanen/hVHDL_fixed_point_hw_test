"""
make_peri.py - generate uart_test.peri.xml (the Interface Designer
periphery) for the Ti60F225 EVM with Efinity's python api

    source ~/efinity/2026.1/bin/setup.sh
    python3 make_peri.py

  pll_refclk  GPIOL_P_18_PLLIN0 (B2), PLL_TL0 external reference 0, 25 MHz
              (ti60-tsemac-linux: my_pll_refclk, my_ddr_pll ext_ref_clock_id 2 = ext 0)
  main_pll    25 MHz -> main_clock 120 MHz (clkout1)
  uart_rx     GPIOL_01  1.8 V LVCMOS  FT4232H channel C
  uart_tx     GPIOL_02  1.8 V LVCMOS
pins from the ti60-tsemac-linux reference design for the EVM
"""
import os
import sys

sys.path.append(os.environ["EFXPT_HOME"] + "/bin")

from api_service.design import DesignAPI

here = os.path.dirname(os.path.abspath(__file__))

design = DesignAPI(False)
design.create("uart_test", "Ti60F225", here, overwrite=True)

# bank voltages as on the EVM : 3.3 V on BR and TL, 1.8 V elsewhere
for bank in design.get_iobank_voltage():
    design.set_iobank_voltage(bank, "3.3" if bank in ("BR", "TL") else "1.8")

pll = "main_pll"
design.create_block(pll, block_type="PLL")
design.gen_pll_ref_clock(pll, pll_res="PLL_TL0", refclk_src="EXTERNAL", refclk_name="pll_refclk", ext_refclk_no="0")
# GPIOL_P_18_PLLIN0, a 1.8 V bank on the EVM
design.set_property("pll_refclk", "IO_STANDARD", "1.8_V_LVCMOS")
# local feedback runs through clkout0 so it sits at 25 MHz * M / N = 100 MHz,
# the core clock comes from clkout1 : vco 4800 MHz / (O 2 * 20) = 120 MHz,
# the same vco / dividers as the ac_in_ac_out_lab_power_supply titanium build
design.set_property(pll, {
    "REFCLK_FREQ": "25"
    , "LOCKED_PIN": "pll_locked"
    , "M": "4", "N": "1", "O": "2"
    , "CLKOUT0_EN": "1", "CLKOUT0_DIV": "24", "CLKOUT0_PIN": "pll_feedback_clock"
    , "CLKOUT1_EN": "1", "CLKOUT1_DIV": "20", "CLKOUT1_PIN": "main_clock"
}, block_type="PLL")
design.calc_pll_clock(pll)
freqs = design.get_property(pll, ["VCO_FREQ", "CLKOUT0_FREQ", "CLKOUT1_FREQ"], block_type="PLL")
print(pll, freqs)
if abs(float(freqs["CLKOUT1_FREQ"]) - 120.0) > 1e-6:
    sys.exit("main_clock is not 120 MHz")

design.create_input_gpio("uart_rx")
design.assign_resource("uart_rx", "GPIOL_01")
design.set_property("uart_rx", {"IO_STANDARD": "1.8_V_LVCMOS", "PULL_OPTION": "WEAK_PULLUP"})

design.create_output_gpio("uart_tx")
design.assign_resource("uart_tx", "GPIOL_02")
design.set_property("uart_tx", "IO_STANDARD", "1.8_V_LVCMOS")

for name in ("pll_refclk", "uart_rx", "uart_tx"):
    print(name, design.get_resource(name))

design.check_design()
design.save()
