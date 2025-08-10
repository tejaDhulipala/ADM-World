from abc import ABC, abstractmethod
from typing import Dict, Tuple

from ..OpenRouterAgent import ADMAgent


class Scenario(ABC):
    def __init__(self, ADM_algorithm: ADMAgent):
        self.ADM_algorithm = ADM_algorithm
    
    @abstractmethod
    def start_environment(self):
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
    def _close_scenario(self):
        pass

    @abstractmethod 
    def get_end_info(self):
        pass