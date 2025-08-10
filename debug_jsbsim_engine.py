from jsbgym.simulation_interface import SimulationInterface

sim = SimulationInterface()
sim.initialize()
# Comprehensive list of JSBSim engine properties to check
jsbsim_c172_engine_properties = [
    "propulsion/engine[0]/set-running",
    "propulsion/engine[0]/starter",
    "propulsion/engine[0]/cutoff",
    "propulsion/engine[0]/mixture",
    "propulsion/engine[0]/throttle",
    "propulsion/engine[0]/propeller-pitch",
    "propulsion/engine[0]/engine-rpm",
    "propulsion/engine[0]/running",
    "propulsion/engine[0]/cranking",
    "propulsion/engine[0]/fuel-flow-rate-pps",
    "propulsion/engine[0]/fuel-flow-gph",
    "propulsion/engine[0]/mp-osi",
    "propulsion/engine[0]/power-hp",
    "propulsion/engine[0]/thrust-lbs",
    "propulsion/engine[0]/propeller-rpm",
    "propulsion/engine[0]/advance-ratio",
    "propulsion/engine[0]/power-coefficient",
    "propulsion/engine[0]/thrust-coefficient",
    "propulsion/engine[0]/helical-tip-mach",
    "propulsion/engine[0]/prop-induced-velocity_fps",
    "propulsion/engine[0]/fuel-consumed-lbs",
    "propulsion/engine[0]/bsfc-lbs_hphr",
    "propulsion/tank[0]/contents-lbs",
    "propulsion/tank[0]/capacity-gal_us",
    "propulsion/tank[0]/pct-full",
    "propulsion/tank[0]/density-lbs_gal",
    "propulsion/tank[0]/external-flow-rate-pps",
    "propulsion/tank[0]/collector-valve",
    "propulsion/tank[0]/priority-valve",
    "propulsion/refuel",
    "propulsion/engine[0]/cht-degf",
    "propulsion/engine[0]/egt-degf",
    "propulsion/engine[0]/oil-temperature-degf",
    "propulsion/engine[0]/oil-pressure-psi",
    "propulsion/engine[0]/boost-cmhg",
    "propulsion/engine[0]/magneto-l",
    "propulsion/engine[0]/magneto-r",
    "propulsion/engine[0]/ignition-switch",
    "propulsion/engine[0]/density-altitude",
    "propulsion/engine[0]/power-altitude-ft",
    "atmosphere/density-altitude",
    "propulsion/engine[0]/serviceable",
    "propulsion/engine[0]/engine-health",
    "propulsion/engine[0]/exhaust-flames",
    "propulsion/engine[0]/carburetor-ice",
    "propulsion/engine[0]/carb-heat",
    "propulsion/engine[0]/starter-torque",
    "propulsion/engine[0]/starter-power-norm",
    "propulsion/engine[0]/combustion"
]

print("=== ALL ENGINE PROPERTIES ===")
print("-" * 50)

for prop in jsbsim_c172_engine_properties:
    value = sim.get_property(prop)
    print(f"{prop}: {value}")