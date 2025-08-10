import copy

from jsbgym.aircraft import c172x
from jsbgym.simulation_interface import SimulationInterface
import jsbgym.properties as prp

# Create original simulation
original_sim = SimulationInterface()
original_sim.initialize()

# Make a deep copy
copied_sim = copy.deepcopy(original_sim)

# Test independence: modify original, check copy is unaffected
original_altitude = copied_sim.sim[prp.altitude_sl_ft]
original_sim.sim[prp.altitude_sl_ft] = 10000  # Change original

# Check copy wasn't affected
assert copied_sim.sim[prp.altitude_sl_ft] == original_altitude
print("✓ Independence test passed")