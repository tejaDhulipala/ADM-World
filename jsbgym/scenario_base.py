from abc import ABC, abstractmethod
from ast import Dict


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
    def set_initial_action(self, actions: Dict):
        pass

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def is_terminated(self, obs: Dict):
        pass

    @abstractmethod
    def is_truncated(self):
        pass

    @abstractmethod
    def close_scenario(self):
        pass