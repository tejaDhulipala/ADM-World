from ast import Dict
from scenario_base import Scenario
from simulation_interface import SimulationInterface

class InstructionFollowingScenario(Scenario):
    valid_subsystems = ["shallow heading hold", "shallow altitude hold", "wings level"]
    def __init__(self, base_instruction: Dict, control_system):
        # Check that the isntruction references the control system
        if "control system" not in base_instruction:
            raise AssertionError("Must include control system")
        # Check that the base instruction contains one of the subsystems 
        i = 0
        for subsystem in base_instruction["control system"]:
            if subsystem not in InstructionFollowingScenario.valid_subsystems:
                raise AssertionError(f"""You must include instructions for a valid subsystem.
                {subsystem} is not a valid subsystems. {InstructionFollowingScenario.valid_subsystems}
                are the valid subsystems.
                """
                )
            i += 1
        if i == 0:
            raise AssertionError("Must include instructions for at least one subsystem")        

        self.simulation_interface = SimulationInterface()
        self.initial_observation = None
    
    def start_environment(self):
        self.initial_observation = self.simulation_interface.initialize()

    def get_initial_observation(self) -> Dict:
        if self.initial_observation == None:
            raise AssertionError("Must start environment first.")
        return self.initial_observation

    def set_initial_action(self, actions: Dict):
        pass

    def run(self):
        pass

    def is_terminated(self, obs: Dict):
        pass

    def is_truncated(self):
        pass

    def close_scenario(self):
        self.simulation_interface.close()