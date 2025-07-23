import jsbsim
import os
import jsbgym

ROOT_DIR = os.path.abspath(jsbsim.__path__[0])
print(ROOT_DIR)
sim = jsbsim.FGFDMExec(root_dir=ROOT_DIR)

plane = jsbgym.aircraft.c172x
sim.load_model(plane.jsbsim_id)

print(sim.get_property_catalog())
print("TESTING SOMETHING")