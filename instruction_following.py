from jsbgym.control_system.control_system_default import ControlInterfaceDefault
from jsbgym.control_system.instruction_following_scenario import InstructionFollowingScenario

base_inst = {
    "control interface": {
        # "shallow heading hold": {
        #     "heading": 0
        # },
        # "shallow altitude hold": {
        #     "altitude": 5000
        # },
        "wings level": {
            "armed": 1
        },
        "shallow pitch hold": {
            "pitch": 0
        }
    },
    "manual": {
        "throttle": 2300,
        "mixture": 0.8
    }

}

scenario = InstructionFollowingScenario(base_inst, ControlInterfaceDefault(), render_mode="graph", max_steps=150)
scenario.start_environment()
scenario.run()
scenario.close_scenario()
print(scenario.get_end_info())