from typing import Dict, Tuple
from .control_system_base import ControlInterfaceBase
from ..scenario_base import Scenario
from ..simulation_interface import SimulationInterface
from .. import properties as prp

class InstructionFollowingScenario(Scenario):
    """
    This scenario is purely for assessing the capabilities of the control interface at following 
    instructions. This isn't a good example of what a scenario should normally look like
    because a scenari should take in a callable that gives instructions, not just
    one initial instruction. The order of using is: init, start_environment, run, close_scenario,
    and get_end_info
    """
    valid_subsystems = ["shallow heading hold", "shallow altitude hold", "wings level"]
    def __init__(self, base_instruction: Dict, control_system: ControlInterfaceBase, render_mode=None, max_steps=300):  
        self.simulation_interface = SimulationInterface(render_mode=render_mode)
        self.control_system = control_system
        self.control_system.set_instruction(base_instruction)
        self.cur_observation = None
        self.end_info_scenario = {}
        self.steps = 0
        self.max_steps = max_steps
    
    def start_environment(self):
        self.cur_observation = self.simulation_interface.initialize()

    def get_initial_observation(self) -> Dict:
        if self.cur_observation == None:
            raise AssertionError("Must start environment first.")
        return self.cur_observation

    def set_inital_instruction(self, actions: Dict):
        pass

    def run(self):
        while not self.is_terminated() and not self.is_truncated():
            actions = self.control_system.action(self.cur_observation)
            self.cur_observation = self.simulation_interface.step(actions)
            self.steps += 1

    def is_terminated(self) -> bool:
        if self.cur_observation[prp.altitude_agl_ft] < 100:
            self.end_info_scenario["end reason"] = "Altitude decreased below 100 ft"
            return True
        if self.cur_observation[prp.ias_kts] > 160:
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
        return self.end_info_scenario, self.control_system.get_eval()