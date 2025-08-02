import jsbgym
import jsbgym.properties as prp
from jsbgym.simulation_interface import SimulationInterface
import matplotlib.pyplot as plt
from math import pi
import numpy as np

aircraft = jsbgym.c172x

sim = SimulationInterface(aircraft=aircraft, render_mode=None) # render mode can be None, flightgear, human, graph, human_fg, or graph_fg
[print(k, v) for k, v in sim.initialize().items()]
print(sim.get_property("inertia/cg-x-in"))
print(sim.get_property("inertia/cg-y-in"))
print(sim.get_property("inertia/cg-z-in"))

num_steps = 0
max_steps = 1000
thrust = []
mixture_poss = []
mixture_cmds = []
while num_steps < max_steps:
    actions = {
        prp.throttle_cmd: 0.8,
        prp.mixture_cmd: 0.0,
        # prp.ap_hold_heading: 1,
        # prp.ap_heading_setpoint: 0,
        # prp.ap_hold_altitude: 1,
        # prp.ap_altitude_setpoint: 6000,
        prp.engine_running: 1,
    }
    obs = sim.step(actions)
    thrust.append(sim.get_property(prp.engine_thrust_lbs))
    mixture_poss.append(sim.get_property("fcs/mixture-pos-norm"))
    mixture_cmds.append(sim.get_property("fcs/mixture-cmd-norm"))
    num_steps += 1
sim.close()

# Commands
plt.figure()
plt.plot(sim.time_series, sim.aileron_cmd, label='Aileron Command')
plt.plot(sim.time_series, np.array(sim.elevator_vars)[:,0], label='Elevator Command')
plt.plot(sim.time_series, np.array(sim.elevator_vars)[:,1], label='Elevator Position Radians')
plt.plot(sim.time_series, np.array(sim.elevator_vars)[:,2], label='AP Elevator Command')
plt.plot(sim.time_series, np.array(sim.elevator_vars)[:,3], label='pitch trim cmd norm')
plt.plot(sim.time_series, sim.rudder_cmd, label='Rudder Command')
plt.xlabel('Simulation Time (s)')
plt.ylabel('Control Command Value')
plt.title('Control Commands vs Simulation Time')
plt.legend()

# Mixture
# Values become nan for some reason
plt.figure()
plt.xlim(0, max(sim.time_series))
plt.plot(sim.time_series, mixture_poss, label='Mixture Position')
plt.plot(sim.time_series, mixture_cmds, label='Mixture Command')
plt.xlabel('Simulation Time (s)')
plt.ylabel('Mixture Values')
plt.legend()
plt.title('Mixture Commands vs Simulation Time')

# Altitude
plt.figure()
plt.plot(sim.time_series, [obs[prp.altitude_sl_ft] for obs in sim.observations], label='Altitude (ft)')
plt.xlabel('Simulation Time (s)')
plt.ylabel('Altitude (ft)')
plt.title('Altitude vs Simulation Time')
plt.legend()


# Pitch
plt.figure()
plt.plot(sim.time_series, np.array([obs[prp.pitch_rad] for obs in sim.observations]) / pi * 180, label='Pitch (degrees)')
plt.xlabel('Simulation Time (s)')
plt.ylabel('Pitch (rad)')
plt.title('Pitch vs Simulation Time')
plt.legend()

# Heading
plt.figure()
plt.plot(sim.time_series, [obs[prp.heading_deg] for obs in sim.observations], label='Heading (deg)')
plt.xlabel('Simulation Time (s)')
plt.ylabel('Heading (deg)')
plt.title('Heading vs Simulation Time')
plt.legend()

# Airspeed
plt.figure()
plt.plot(sim.time_series, [obs[prp.cas_kts] for obs in sim.observations], label='Airspeed (deg)')
plt.xlabel('Simulation Time (s)')
plt.ylabel('Airspeed (deg)')
plt.title('Airspeed vs Simulation Time')
plt.legend()


# Thrust
plt.figure()
plt.plot(sim.time_series, thrust, label='thrust')
plt.xlabel('Simulation Time (s)')
plt.ylabel('thrust')
plt.title('thrust vs Simulation Time')
plt.legend()
plt.show()


input("Enter to exit plotting: ")