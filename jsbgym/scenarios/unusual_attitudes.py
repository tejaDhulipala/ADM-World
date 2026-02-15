from math import pi
from typing import Dict, Tuple
import random

from ..aircraft import c172x
from ..control_system.control_interface_base import ControlInterfaceBase
from .scenario_base import Scenario
from ..simulation_interface import SimulationInterface
from .. import properties as prp

class UnusualAttitudes(Scenario):
    """
    IFR flight scenario
    """
    valid_subsystems = ["shallow heading hold", "shallow altitude hold", "wings level"]
    def __init__(self, control_interface: ControlInterfaceBase, render_mode=None, max_steps=300):  
        self.simulation_interface = None
        self.control_interface = control_interface
        self.cur_observation = None
        self.end_info_scenario = {}
        self.steps = 0
        self.render_mode = render_mode
        self.max_steps = max_steps
    
    def start_environment(self):
        ics = {
            prp.initial_altitude_msl_ft: random.uniform(300, 1000),
            prp.initial_longitude_geoc_deg: 97.5195,
            prp.initial_latitude_geod_deg: 35.4689,
            prp.initial_p_radps: 3 / 57,
            prp.initial_q_radps: -2/57,
            prp.initial_r_radps: 0,
            prp.initial_heading_deg: random.uniform(0, 360),
            prp.initial_pitch_deg: random.uniform(-60, -10),
            prp.initial_roll_deg: random.uniform(15, 60),
            "ic/vc-kts": random.uniform(c172x.Vno, c172x.Vne + 75),
        }
        self.simulation_interface = SimulationInterface(c172x, 
        ics, [], control_agent_interaction_freq=5, render_mode=self.render_mode)
        self.cur_observation = self.simulation_interface.initialize()

    def get_initial_observation(self) -> Dict:
        if self.cur_observation == None:
            raise AssertionError("Must start environment first.")
        return self.cur_observation

    def run(self):
        while not self.is_terminated() and not self.is_truncated():
            actions = self.control_interface.action(self.cur_observation)
            self.cur_observation = self.simulation_interface.step(actions)
            self.steps += 1

    def is_terminated(self) -> bool:
        if self.cur_observation[prp.altitude_agl_ft] < 100:
            self.end_info_scenario["end reason"] = "Altitude decreased below 100 ft"
            return True
        if self.cur_observation[prp.cas_kts] > 160:
            self.end_info_scenario["end reason"] = "Exceeded 160kts aka Vne"
            return True
        return False

    def is_truncated(self) -> bool:
        if self.steps > self.max_steps:
            self.end_info_scenario["end reason"] = f"Truncated. Number of steps exceeded {self.max_steps}"
            return True
        return False

    def close_scenario(self):
        self.simulation_interface.close()
    
    def get_end_info(self) -> Tuple[Dict, Dict]:
        self.end_info_scenario["num steps"] = self.steps
        return self.end_info_scenario, self.control_interface.get_eval()