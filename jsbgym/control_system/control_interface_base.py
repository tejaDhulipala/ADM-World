from abc import ABC, abstractmethod
from pickle import DICT
from typing import Dict
from ..simulation_interface import SimulationInterface


class ControlInterfaceBase(ABC):
    """
    The purpose of the control interface is to parse instructions from the decision making 
    algorithm and return an action for the Simulation Interface to parse.

    An example instruction looks like this:
    control interface:
        shallow heading hold:
            heading:
        shallow altitude hold:
            altitude:
        wings level:
            armed: 
    manual:
        throttle:
        mixture:
    until:
        1: 
        2:
        3:
    """

    def __init__(self):
        self.current_instruction = None
    
    @abstractmethod
    def set_instruction(self, instruction: Dict, sim: SimulationInterface):
        self.current_instruction = instruction

    @abstractmethod
    def action(self, sim: SimulationInterface) -> Dict:
        pass

    @abstractmethod
    def get_eval(self) -> Dict:
        pass