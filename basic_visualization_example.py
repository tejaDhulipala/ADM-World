import jsbgym
import jsbgym.properties as prp
from jsbgym.simulation_interface import SimulationInterface


aircraft = jsbgym.c172x

sim = SimulationInterface(render_mode="flightgear") # render mode can be None, flightgear, human, graph, human_fg, or graph_fg
[print(k, v) for k, v in sim.initialize().items()]

num_steps = 0
max_steps = 1000
while num_steps < max_steps:
    actions = {
        prp.throttle_cmd: 0.8,
        prp.mixture_cmd: 0.8,
        prp.ap_hold_altitude: 1,
        # prp.ap_altitude_setpoint: 1000,
        prp.ap_hold_heading: 1,
        prp.ap_heading_setpoint: 20
    }
    obs = sim.step(actions)
    if not num_steps % 30:
        print(obs[prp.altitude_sl_ft])
    num_steps += 1
sim.close()