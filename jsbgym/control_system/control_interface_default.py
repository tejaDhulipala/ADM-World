from typing import Dict, Tuple

from .ha_flight_pid import FGAPControlSubsystem, ManualPropertiesSubsystem, StraightAndLevelSubsystem
from ..simulation_interface import SimulationInterface
from .control_interface_base import ControlInterfaceBase
from .. import properties as prp
import warnings

class ControlInterfaceDefault(ControlInterfaceBase):
    """
    The purpose of the control interface is to parse instructions from the decision making 
    algorithm and return an action for the Simulation Interface to parse. This is the default
    action. 

    An example instruction looks like this:
    control interface:
        heading:
        altitude:
        wings level:
    manual:
        throttle:
        mixture:
    until:
        1: 
        2:
        3:
    
    If a control subsystem is not mentioned, it is disarmed. If it is mentioned, it is armed.
    """
    ACCEPTABLE_HEADING_ERR = 10
    ACCEPTABLE_ALTITUDE_ERR = 100

    def __init__(self):
        self.current_instruction = None
        self.straight_and_level_subsystem = StraightAndLevelSubsystem()
        self.ha_control_subsystem = FGAPControlSubsystem()
        self.manual_subsystem = ManualPropertiesSubsystem()
        self.cur_eval = {
            "max alt overshoot": 0,
            "max hdg overshoot": 0,
            "max load factor": 0,
            "avg load factor": 0,
            "min airspeed": 1000,
            "max airspeed": -1,
            "avg airspeed": 0,
            "avg alt steady state error": 0,
            "avg hdg steady state error": 0,
            "time to first contact s": 0,
            "max man time mins": -1,
            "max time mins": -1 / -1 / 60
        }
        self.instructions = []
        self.evals = []
        self.cur_alt_change = 0
        self.cur_hdg_change = 0
        self.steps_total = 0
        self.instr_steps = 0

    def get_instructions(self) -> Dict:
        return self.instructions
    
    def set_instruction(self, instruction, sim: SimulationInterface):
        if self.current_instruction is not None:
            self.instructions.append(self.current_instruction)
            self.cur_eval["avg airspeed"] /= self.instr_steps
            self.cur_eval["avg load factor"] /= self.instr_steps
            self.cur_eval["avg alt steady state error"] /= self.instr_steps
            self.cur_eval["avg hdg steady state error"] /= self.instr_steps
            self.evals.append(self.cur_eval) 
        self.current_instruction = instruction
        self.instructions.append(instruction)
        
        # Set variables for the new instruction
        self.cur_eval = self.cur_eval = {
            "max alt overshoot": 0,
            "max hdg overshoot": 0,
            "max load factor": 0,
            "avg load factor": 0,
            "min airspeed": 1000,
            "max airspeed": -1,
            "avg airspeed": 0,
            "avg alt steady state error": 0,
            "avg hdg steady state error": 0,
            "time to first contact s": 0,
            "max man time mins": -1,
            "max time mins": -1 / -1 / 60
        }
        try:
            self.cur_alt_change = instruction["control interface"]["altitude"] - sim.get_property(prp.altitude_sl_ft)
            self.cur_hdg_change = instruction["control interface"]["heading"] - sim.get_property(prp.heading_deg)
        except KeyError:
            warnings.warn("No altitude or heading hold in instruction, maintaining current altitude and heading")
            self.cur_alt_change = 0
            self.cur_hdg_change = 0
        self.instr_steps = 0
        max_maneuver_time_mins = max(abs(self.cur_alt_change) / 500, abs(self.cur_hdg_change) / 180) * 1.5
        self.max_man_steps = max_maneuver_time_mins * 60 * sim.control_agent_interaction_freq

    def action(self, sim: SimulationInterface) -> Dict:
        """
        The priority is first wings level, then heading and altitude. Manual overrides all.
        """
        actions = {}
        try:
            des_alt = self.current_instruction["control interface"]["altitude"]
        except KeyError:
            des_alt = sim.get_property(prp.altitude_sl_ft)
            warnings.warn("No altitude hold in instruction, maintaining current altitude")

        try:
            des_hdg = self.current_instruction["control interface"]["heading"] % 360
        except KeyError:
            des_hdg = sim.get_property(prp.heading_deg)
            warnings.warn("No heading hold in instruction, maintaining current heading")
        
        for prop, val in self.ha_control_subsystem.action(sim, des_alt, des_hdg).items():
                actions[prop] = val

        if "wings level" in self.current_instruction["control interface"]:
            for prop, val in self.straight_and_level_subsystem.action(self.current_instruction["control interface"]["wings level"]).items():
                actions[prop] = val
        else:
            for prop, val in self.straight_and_level_subsystem.action(0).items():
                actions[prop] = val

        # Manual section
        # Must override all autopilot stuff
        if "manual" in self.current_instruction:
            for (prop, prop_val) in self.current_instruction["manual"].items():
                jsb_prop, jsb_val = self.manual_subsystem.convert_to_jsbsim_prop_val(prop, prop_val)
                actions[jsb_prop] = jsb_val
        
        self.update_eval(sim, des_alt, des_hdg)

        self.steps_total += 1
        self.instr_steps += 1
        
        return actions
    
    def update_eval(self, sim: SimulationInterface, des_alt, des_hdg):
        # Edit max alt overshoot
        alt_error = abs(des_alt - sim.get_property(prp.altitude_sl_ft))
        hdg_error = min(abs(des_hdg - sim.get_property(prp.heading_deg)), 360 - abs(des_hdg - sim.get_property(prp.heading_deg)))
        if self.cur_alt_change < 0 and sim.get_property(prp.altitude_sl_ft) < des_alt:
            self.cur_eval["max alt overshoot"] = max(self.cur_eval["max alt overshoot"], alt_error)
        elif self.cur_alt_change > 0 and sim.get_property(prp.altitude_sl_ft) > des_alt:
            self.cur_eval["max alt overshoot"] = max(self.cur_eval["max alt overshoot"], alt_error)
        # Edit max hdg overshoot
        if self.cur_hdg_change < 0 and sim.get_property(prp.heading_deg) < des_hdg:
            self.cur_eval["max hdg overshoot"] = max(self.cur_eval["max hdg overshoot"], hdg_error)
        elif self.cur_hdg_change > 0 and sim.get_property(prp.heading_deg) > des_hdg:
            self.cur_eval["max hdg overshoot"] = max(self.cur_eval["max hdg overshoot"], hdg_error)
        # Max load factor
        self.cur_eval["max load factor"] = max(self.cur_eval["max load factor"], abs(sim.get_property(prp.load_factor)))
        # Avg load factor. I'm not actually calculating the avg yet for simplicity
        self.cur_eval["avg load factor"] += abs(sim.get_property(prp.load_factor))
        # Min airspeed
        self.cur_eval["min airspeed"] = min(self.cur_eval["min airspeed"], sim.get_property(prp.cas_kts))
        # max airspeed
        self.cur_eval["max airspeed"] = max(self.cur_eval["max airspeed"], sim.get_property(prp.cas_kts))
        # avg airspeed, but sum for now
        self.cur_eval["avg airspeed"] += sim.get_property(prp.cas_kts)
        # avg alt steady state error
        if self.instr_steps > self.max_man_steps:
            self.cur_eval["avg alt steady state error"] += alt_error
        # avg hdg steady state error
        if self.instr_steps > self.max_man_steps:
            self.cur_eval["avg hdg steady state error"] += hdg_error
        # Time to first contact
        if alt_error < self.ACCEPTABLE_ALTITUDE_ERR and hdg_error < self.ACCEPTABLE_HEADING_ERR and not \
                self.cur_eval["time to first contact s"]:
                self.cur_eval["time to first contact s"] = self.instr_steps / sim.control_agent_interaction_freq
    
    def get_eval(self) -> Tuple[Dict, Dict]:
        """
        Returns a tuple of all of the instructions and the eval.
        """
        return self.instructions, self.evals