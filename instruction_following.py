from jsbgym.scenarios.instruction_following import *  
from jsbgym.OpenRouterAgent import OpenRouterAgent

class DemoADMAlgorithm:
    def __init__(self):
        pass

    def get_decision(self, system_prompt, cur_prompt):
        return {
        {
            "control interface": {
                "heading": 330,
                "altitude": 3000,
            },
            "manual": {
            }
        }
    }   

scenario = InstructionFollowing(
    ADM_algorithm=DemoADMAlgorithm(),
    case=InstructionFollowingCases.CLIMB_TO_ALTITUDE,
    render_mode="",
    custom_tracking_vars=[
        prp.v_down_fps,
        prp.propeller_rpm
    ]
)
scenario.start_environment()
scenario.run()
print(scenario.get_end_info())
scenario.plot_tracking_variables()