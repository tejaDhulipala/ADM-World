from typing import Dict, Tuple
from .control_system_base import ControlInterfaceBase
from .. import properties as prp

class ControlInterfaceDefault(ControlInterfaceBase):
    """
    The purpose of the control interface is to parse instructions from the decision making 
    algorithm and return an action for the Simulation Interface to parse. This is the default
    action. 

    An example instruction looks like this:
    control interface:
        shallow heading hold:
            heading:
        shallow altitude hold:
            altitude:
        shallow pitch hold:
            pitch: 0
        wings level:
            armed:
    manual:
        throttle:
        mixture:
    until:
        1: 
        2:
        3:
    
    If a control subsystem is not mentioned, it is disarmed. If it is mentioned, it is armed.
    """
    acceptable_heading_dev = 5
    acceptable_altitude_dev = 50

    def __init__(self, max_throttle=2700):
        super() # create self.current_instruction and holds self.set_instruction()
        self.altitude_hold_subsystem = AltitudeHoldSubsystem()
        self.heading_hold_subsystem = HeadingHoldSubsystem()
        self.straight_and_level_subsystem = StraightAndLevelSubsystem()
        self.manual_subsystem = ManualPropertiesSubsystem(max_throttle)
        self.eval = {
            "steps outside acceptable heading": 0,
            "steps outside acceptable altitude": 0,
            "avg altitude error": 0,
            "avg heading error": 0,
            "min airspeed": None,
            "max airspeed": None,
            "avg sideslip angle": 0,
        }
        self.steps = 0

    def action(self, observation: Dict) -> Dict:
        """
        The priority is first altitude hold, then heading hold, then attitude hold / wings level.
        """
        actions = {}
        if "wings level" in self.current_instruction["control interface"]:
            for prop, val in self.straight_and_level_subsystem.action(1).items():
                actions[prop] = val
        else:
            actions[prp.ap_hold_straight_and_level] = 0
        
        if "shallow heading hold" in self.current_instruction["control interface"]:
            h_setpoint = self.current_instruction["control interface"]["shallow heading hold"]["heading"]
            for prop, val in self.heading_hold_subsystem.action(1, h_setpoint).items():
                actions[prop] = val
        else:
            for prop, val in self.heading_hold_subsystem.action(0, observation[prp.heading_deg]).items():
                actions[prop] = val
        
        if "shallow altitude hold" in self.current_instruction["control interface"]:
            h_setpoint = self.current_instruction["control interface"]["shallow altitude hold"]["altitude"]
            for prop, val in self.altitude_hold_subsystem.action(1, h_setpoint, observation[prp.altitude_sl_ft], observation[prp.sim_dt]).items():
                actions[prop] = val
        else:
            for prop, val in self.altitude_hold_subsystem.action(0, observation[prp.altitude_sl_ft], observation[prp.altitude_sl_ft], observation[prp.sim_dt]).items():
                actions[prop] = val
        
        if "shallow pitch hold" in self.current_instruction["control interface"]:
            p_setpoint = self.current_instruction["control interface"]["shallow pitch hold"]["pitch"]
            for prop, val in self.pitch_hold_subsystem.action(1, p_setpoint, observation[prp.pitch_rad], observation[prp.sim_dt]).items():
                actions[prop] = val
        else:
            for prop, val in self.pitch_hold_subsystem.action(1, 0, observation[prp.pitch_rad], observation[prp.sim_dt]).items():                
                actions[prop] = val

        # Manual section
        # Must override all ap stuff
        if "manual" in self.current_instruction:
            for (prop, prop_val) in self.current_instruction["manual"].items():
                jsb_prop, jsb_val = self.manual_subsystem.convert_to_jsbsim_prop_val(prop, prop_val)
                actions[jsb_prop] = jsb_val
        
        self.update_eval(observation)
        
        return actions
    
    def update_eval(self, observation: Dict):
        self.steps += 1

        if "shallow heading hold" in self.current_instruction["control interface"]:
            err = abs(observation[prp.heading_deg] - self.current_instruction["control interface"]["shallow heading hold"]["heading"])
            if  err > ControlInterfaceDefault.acceptable_heading_dev:
                self.eval["steps outside acceptable heading"] += 1
            self.eval["avg heading error"] += err

        
        if "shallow altitude hold" in self.current_instruction["control interface"]:
            err = abs(observation[prp.altitude_sl_ft] - self.current_instruction["control interface"]["shallow altitude hold"]["altitude"])
            if  err > ControlInterfaceDefault.acceptable_altitude_dev:
                self.eval["steps outside acceptable altitude"] += 1   
            self.eval["avg altitude error"] += err
         
        if self.eval["min airspeed"] ==  None:
            self.eval["min airspeed"] = observation[prp.cas_kts]
        else: 
            self.eval["min airspeed"] = min(self.eval["min airspeed"], observation[prp.cas_kts])
        
        if self.eval["max airspeed"] ==  None:
            self.eval["max airspeed"] = observation[prp.cas_kts]
        else: 
            self.eval["max airspeed"] = max(self.eval["max airspeed"], observation[prp.cas_kts])
        
        self.eval["avg sideslip angle"] += abs(observation[prp.sideslip_deg])
    
    def get_eval(self):
        eval = self.eval
        eval["avg altitude error"] /= self.steps
        eval["avg heading error"] /= self.steps
        eval["avg sideslip angle"] /= self.steps
        return eval


class AltitudeHoldSubsystem:
    def __init__(self):
        pass

    def action(self, altitude_setpoint) -> Dict:
        return {prp.ap_hold_altitude: 1,
            prp.ap_altitude_setpoint: altitude_setpoint
            }
               
class HeadingHoldSubsystem:
    def __init__(self):
        pass

    def action(self, heading_setpoint) -> Dict:
        return {
            prp.ap_hold_heading: 1,
            prp.ap_heading_setpoint: heading_setpoint
        }

class StraightAndLevelSubsystem:
    def __init__(self):
        pass

    def action(self, armed):
        return {
            prp.ap_hold_straight_and_level: armed,
        }

class ManualPropertiesSubsystem:
    def __init__(self, max_throttle):
        self.max_throttle = max_throttle
    
    def convert_to_jsbsim_prop_val(self, prop_name, prop_val) -> Tuple:
        match prop_name:
            case "throttle":
                return prp.throttle_cmd, prop_val / self.max_throttle
            case "mixture":
                return prp.mixture_cmd, prop_val
            case _:
                print("Command doesn't exist")
                return None, None

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0.0, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits  # (min, max)
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = None

    def reset(self):
        """Reset the PID controller state."""
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = None

    def compute(self, measurement: float, dt: float = None) -> float:
        """
        Compute the PID control output.
        Args:
            measurement: The current measured value.
            dt: Time step (if None, derivative term is not used).
        Returns:
            Control output after applying PID formula and output limits.
        """
        error = self.setpoint - measurement
        self._integral += error * (dt if dt is not None else 1.0)
        derivative = 0.0
        if dt is not None and dt > 0:
            derivative = (error - self._last_error) / dt
        output = (
            self.kp * error +
            self.ki * self._integral +
            self.kd * derivative
        )
        # Apply output limits
        min_out, max_out = self.output_limits
        if min_out is not None:
            output = max(min_out, output)
        if max_out is not None:
            output = min(max_out, output)
        self._last_error = error
        return output 
