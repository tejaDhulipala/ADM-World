from abc import ABC, abstractmethod
from typing import Dict, Tuple


class Scenario(ABC):
    def __init__(self, ADM_algorithm: callable):
        self.ADM_algorithm = ADM_algorithm
    
    @abstractmethod
    def start_environment(self):
        pass

    @abstractmethod
    def get_initial_observation(self) -> Dict:
        pass

    @abstractmethod
    def set_inital_instruction(self, instruction: Dict):
        pass

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def is_terminated(self) -> bool:
        pass

    @abstractmethod
    def is_truncated(self) -> bool:
        pass

    @abstractmethod
    def close_scenario(self):
        pass

    @abstractmethod 
    def get_end_info(self) -> Tuple[Dict, Dict]:
        """
        Returns the end information for the whole scenario first, then for only the control interface
        next.
        """
        pass