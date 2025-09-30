from ..simulation_interface import SimulationInterface
from .. import properties as prp
from numpy import clip
from .pid_controller import PIDController
from typing import Dict, Tuple
import warnings

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

class FGAPControlSubsystem:
    def __init__(self) -> None:
        self.heading_subsystem = HeadingHoldSubsystem()
        self.altitude_subsystem = AltitudeHoldSubsystem()
        self.steps = 0

    def action(self, sim: SimulationInterface, des_alt, des_hdg):
        assert des_hdg <= 360
        actions = {}
        for prop, val in self.heading_subsystem.action(des_hdg).items():
            actions[prop] = val
        for prop, val in self.altitude_subsystem.action(des_alt).items():
            actions[prop] = val
        
        # Set the throttle and mixture 
        alt_change = des_alt - sim.get_property(prp.altitude_sl_ft)
        if  alt_change >= 150:
            actions[prp.throttle_cmd] = ManualPropertiesSubsystem.cessna_rpm_to_throttle_cmd(3000)
        elif alt_change >= -150:
            actions[prp.throttle_cmd] = ManualPropertiesSubsystem.cessna_rpm_to_throttle_cmd(2300)
        else:
            actions[prp.throttle_cmd] = ManualPropertiesSubsystem.cessna_rpm_to_throttle_cmd(1800)
        actions[prp.mixture_cmd] = 0.8

        self.steps += 1

        return actions

class ManualPropertiesSubsystem:
    def __init__(self):
        pass
    
    def convert_to_jsbsim_prop_val(self, prop_name, prop_val) -> Tuple:
        match prop_name:
            case "throttle":
                return prp.throttle_cmd, ManualPropertiesSubsystem.cessna_rpm_to_throttle_cmd(prop_val)
            case "mixture":
                return prp.mixture_cmd, prop_val
            case _:
                warnings.warn("Command doesn't exist")
                return None, None
    
    @staticmethod
    def cessna_rpm_to_throttle_cmd(rpm):
        throttle_cmds = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0]
        rpms = [896, 951, 1008, 1069, 1127, 1188, 1253, 1324, 1403, 1490, 1588, 1697, 1818, 1953, 2103, 2262, 2436, 2611, 2766, 2828, 2845]
        if rpm > rpms[-1]:
            return 1.0
        if rpm < rpms[0]:
            return 0.0
        for i in range(len(rpms)-1):
            if rpms[i] <= rpm <= rpms[i+1]:
                return throttle_cmds[i] + (throttle_cmds[i+1] - throttle_cmds[i]) * (rpm - rpms[i]) / (rpms[i+1] - rpms[i])