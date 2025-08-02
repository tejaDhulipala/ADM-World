from ..simulation_interface import SimulationInterface
from .. import properties as prp
from numpy import clip
from .control_system_default import HeadingHoldSubsystem

class HAPIDControlSubsystem:
    def __init__(self) -> None:
        self.heading_subsystem = HeadingHoldSubsystem()
        self.altitude_subsystem = AltitudePIDSubsystem()
        self.num_steps = 0
        self.trim_throttle = None
        self.trim_pitch = None

    def action(self, sim: SimulationInterface, des_alt, des_hdg):
        # print(f"aero max rad: {sim.get_property('aero/alpha-max-rad')}")
        # print(f"aero-min-rad: {sim.get_property('aero/alpha-min-rad')}")
        # print(f"cas-kts: {sim.get_property(prp.cas_kts)}")
        # print(f"ic/gamma-deg aka flight path angle: {sim.get_property('ic/gamma-deg')}")
        # print(f"ic/vc-kts: {sim.get_property('ic/vc-kts')}")
        # print(f"ic/h-sl-ft: {sim.get_property('ic/h-sl-ft')}")
        assert des_hdg <= 360
        actions = {}

        # Do stuff based on the number of steps
        if self.num_steps == 0:
            actions["simulation/do_simple_trim"] = 0
        if self.num_steps == 1:
            self.trim_throttle = sim.get_property(prp.throttle_cmd)
            self.trim_pitch = sim.get_property(prp.pitch_rad)
        
        # Set the throttle and mixture   
        actions[prp.mixture_cmd] = 0.99

        # Set flight commands
        for prop, val in self.heading_subsystem.action(des_hdg).items():
            actions[prop] = val
        for prop, val in self.altitude_subsystem.action(sim, des_alt).items():
            actions[prop] = val
        
        # Increment num steps
        self.num_steps += 1

        return actions

class AltitudePIDSubsystem:
    def __init__(self) -> None:
        self.alt_hold_pid = PIDController(0, 0, 0)
        self._last_time = None
    
    def action(self, sim: SimulationInterface, des_alt):
        actions = {
            prp.elevator_cmd: 0
        }
        
        if self._last_time == None:
            self._last_time = sim.get_property(prp.sim_time_s)
        
        dt = sim.get_property(prp.sim_time_s) - self._last_time
        elevator_cmd = 0
        if abs(des_alt - sim.get_property(prp.altitude_sl_ft)) < 100:
            elevator_cmd = self.alt_hold_pid.compute(sim.get_property(prp.altitude_sl_ft), des_alt, dt)
        
        # Change variables
        self._last_time = sim.get_property(prp.sim_time_s)
        actions[prp.elevator_cmd] = elevator_cmd

        return actions


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._last_error = 0.0
        self._integral = 0.0

    def reset(self):
        """Reset the PID controller state."""
        self._last_error = 0.0
        self._integral = 0.0

    def compute(self, measurement: float, setpoint, dt: float) -> float:
        """
        Compute the PID control output.
        Args:
            measurement: The current measured value.
            dt: Time step (if 0, derivative term is not used).
        Returns:
            Control output after applying PID formula and output limits.
        """
        error = setpoint - measurement
        self._integral += error * dt
        if dt != 0:
            derivative = (error - self._last_error) / dt
        else: 
            derivative = 0

        return self.kp * error + self.ki * self._integral + self.kd * derivative 