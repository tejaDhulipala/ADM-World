from ..simulation_interface import SimulationInterface
from .. import properties as prp
from numpy import clip
from .control_system_default import AltitudeHoldSubsystem

class PIDControlSubsystem:
    def __init__(self) -> None:
        self.heading_subsystem = HeadingPIDControlSubsystem()
        self.altitude_subsystem = AltitudeHoldSubsystem()

    def action(self, sim: SimulationInterface, des_alt, des_hdg):
        assert des_hdg <= 360
        actions = {}
        for prop, val in self.heading_subsystem.action(sim, des_hdg).items():
            actions[prop] = val
        for prop, val in self.altitude_subsystem.action(des_alt).items():
            actions[prop] = val
        
        # Set the throttle and mixture 
        alt_change = des_alt - sim.get_property(prp.altitude_sl_ft)
        if  alt_change >= 150:
            actions[prp.throttle_cmd] = 1.0
        elif alt_change >= -150:
            actions[prp.throttle_cmd] = 2300 / 2700
        else:
            actions[prp.throttle_cmd] = 1800 / 2700
        actions[prp.mixture_cmd] = 0.8

        return actions

class HeadingPIDControlSubsystem:
    def __init__(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_roll_error_lag = 0.0
        self.aileron_roll_pid = PIDController(50, 5, 17)
        self._last_time = None

    def action(self, sim: SimulationInterface, des_hdg):
        if self._last_time == None:
            self._last_time = sim.get_property(prp.sim_time_s)

        actions = {
            "fcs/aileron-cmd-norm": 0.0
        }

        # Desired roll position
        desired_roll_position = 0

        dt = self.sim.get_property(prp.sim_time_s) - self._last_time
        aileron_cmd = self.aileron_roll_pid.compute(self.sim.get_property(prp.roll_phi_rad), desired_roll_position, dt)
        actions

    
    
    def reset_integral(self):
        self.integral = 0

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

    def compute(self, measurement: float, setpoint, dt: float = None) -> float:
        """
        Compute the PID control output.
        Args:
            measurement: The current measured value.
            dt: Time step (if None, derivative term is not used).
        Returns:
            Control output after applying PID formula and output limits.
        """
        error = setpoint - measurement
        self._integral += error * dt
        derivative = (error - self._last_error) / dt

        return self.kp * error + self.ki * self._integral + self.kd * derivative 