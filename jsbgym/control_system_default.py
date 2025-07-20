from ast import Dict
from control_system_base import ControlSystemBase, SubsystemBase

class ControlSystemDefault(ControlSystemBase):
    """
    The purpose of the control system is to parse instructions from the decision making 
    algorithm and return an action for the Simulation Interface to parse. This is the default
    action. 

    An example instruction looks like this:
    control system:
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
    
    If a control subsystem is not mentioned, it is disarmed. If it is mentioned, it is armed.
    """

    def action(self, observation: Dict) -> Dict:
        """
        The priority is first altitude hold, then heading hold, then attitude hold / wings level.
        """
        pass
 
class AltitudeHoldSubsystem(SubsystemBase):
    def __init__(self, armed=0, set_altitde):
        super().__init__()