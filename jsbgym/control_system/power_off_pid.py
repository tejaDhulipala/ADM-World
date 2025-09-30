from numpy import clip
from .ha_flight_pid import HeadingHoldSubsystem, PIDController
from ..simulation_interface import SimulationInterface
from .. import properties as prp

class PowerOffControlSubsystemRawPID:
    def __init__(self) -> None:
        self.heading_subsystem = HeadingHoldSubsystem() 
        self.num_steps = 0
        self.airspeed_pid = PIDController(0.005, 0.001, 0)
        self.trim_throttle = None
        self.trim_pitch = None

    def action(self, sim: SimulationInterface, des_airspeed, des_hdg):
        assert 0 <= des_hdg <= 360
        actions = {}

        # Do stuff based on the number of steps
        if self.num_steps == 0:
            actions["simulation/do_simple_trim"] = 0
        
        # Set the elevator command
        pid_output = self.airspeed_pid.compute(sim.get_property(prp.cas_kts), des_airspeed, sim.get_property(prp.sim_dt) * sim.control_agent_interaction_freq)
        pid_output = clip(pid_output, -0.5, 0.5)
        actions[prp.elevator_cmd] = pid_output

        # Set flight commands
        for prop, val in self.heading_subsystem.action(des_hdg).items():
            actions[prop] = val
        
        # Increment num steps
        self.num_steps += 1

        return actions


class PowerOffControlSubsystemDiffPID:
    def __init__(self) -> None:
        self.heading_subsystem = HeadingHoldSubsystem() 
        self.num_steps = 0
        self.airspeed_rate_pid = PIDController(0.5, 0, 0)
        self.elevator_pid = PIDController(0.03, 0.0175, 0)
        self.actual_rocs = []
        self.rocs = []
        self.times= []
        self._last_airspeed = 0

    def action(self, sim: SimulationInterface, des_airspeed, des_hdg):
        assert 0 <= des_hdg <= 360
        actions = {}

        # Do stuff based on the number of steps
        if self.num_steps == 0:
            self._last_airspeed = sim.get_property(prp.cas_kts)
            actions["simulation/do_simple_trim"] = 0
        
        # Set the elevator command
        dt = sim.get_property(prp.sim_dt) * sim.control_agent_interaction_freq
        # Desired rate of change of calibrated airspeed in knots per second
        cas_roc = self.airspeed_rate_pid.compute(sim.get_property(prp.cas_kts), des_airspeed, dt)
        cas_roc = clip(cas_roc, -5, 5)
        # Now that we have the desired rate of change of airspeed, we should change the pitch
        # accordingly. 
        airspeed_rate = (sim.get_property(prp.cas_kts) - self._last_airspeed) / dt
        elev_pid = self.elevator_pid.compute(airspeed_rate, cas_roc, dt)
        # The elevator should move no more than -0.5 to 0.5
        elev_pid = clip(elev_pid, -0.5, 0.5)
        
        actions[prp.elevator_cmd] = elev_pid

        # Set flight commands
        for prop, val in self.heading_subsystem.action(des_hdg).items():
            actions[prop] = val
        
        # Increment num steps
        self.num_steps += 1
        self._last_airspeed = sim.get_property(prp.cas_kts)
        self.rocs.append(cas_roc)
        self.actual_rocs.append(airspeed_rate)
        self.times.append(sim.get_property(prp.sim_time_s))

        return actions
    
    def get_data(self):
        return {"ROCs": self.rocs, "Actual ROCs": self.actual_rocs}

    def reset(self):
        self.heading_subsystem = HeadingHoldSubsystem() 
        self.num_steps = 0
        self.airspeed_rate_pid = PIDController(0.5, 0, 0)
        self.elevator_pid = PIDController(0.03, 0.0175, 0)
        self.actual_rocs = []
        self.rocs = []
        self.times= []
        self._last_airspeed = 0