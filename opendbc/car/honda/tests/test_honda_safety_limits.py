import os
import re
import unittest

from opendbc.car.common.basedir import BASEDIR

HONDA_DIR = os.path.join(BASEDIR, "honda")  # BASEDIR is opendbc/car
HONDA_SAFETY = os.path.join(BASEDIR, "..", "safety", "modes", "honda.h")
STOCK_BOSCH_MAX_GAS = 2000


def panda_bosch_max_gas() -> int:
  src = open(HONDA_SAFETY).read()
  block = re.search(r"HONDA_BOSCH_LONG_LIMITS\s*=\s*\{(.*?)\};", src, re.S)
  assert block, "HONDA_BOSCH_LONG_LIMITS not found in honda.h"
  return int(re.search(r"\.max_gas\s*=\s*(\d+)", block.group(1)).group(1))


class TestHondaSafetyLimits(unittest.TestCase):
  def test_bosch_max_gas_is_stock(self):
    # MVL raised this to 2200; restored to the stock limit. Raising it again needs an explicit safety review.
    self.assertEqual(panda_bosch_max_gas(), STOCK_BOSCH_MAX_GAS)

  def test_gas_lookups_within_panda_limit(self):
    # Any Bosch gas command above the panda limit gets ACC_CONTROL blocked, and a gap in ACC_CONTROL latches
    # CRUISE_FAULT in the VSA. Every lookup assignment must stay within the panda's max_gas.
    limit = panda_bosch_max_gas()
    found = []
    for fn in sorted(os.listdir(HONDA_DIR)):
      if fn.endswith(".py"):
        for m in re.finditer(r"BOSCH_GAS_LOOKUP_V\s*=\s*\[([^\]]*)\]", open(os.path.join(HONDA_DIR, fn)).read()):
          vals = [float(v) for v in m.group(1).split(",")]
          found.append((fn, vals))
          self.assertLessEqual(max(vals), limit, f"{fn}: BOSCH_GAS_LOOKUP_V {vals} exceeds panda max_gas {limit}")
    self.assertGreater(len(found), 1, "expected to find the default lookup and per-car overrides")


if __name__ == "__main__":
  unittest.main()
