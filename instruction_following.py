from jsbgym.scenarios.instruction_following import *  
from jsbgym.OpenRouterAgent import OpenRouterAgent
import json

class DemoADMAlgorithm:
    def __init__(self):
        self.num_calls = 1

    def get_decision(self, system_prompt, cur_prompt):
        system_prompt_str = OpenRouterAgent.format_system_prompt(system_prompt)
        cur_prompt_str = json.dumps(cur_prompt, indent=2, ensure_ascii=False)
        if self.num_calls == 1:
            print("=" * 20 + " System Prompt " + "=" * 20)
            print(system_prompt_str)
        print("=" * 20 + " Current Prompt " + "=" * 20)
        print(cur_prompt_str)
        self.num_calls += 1
        if self.num_calls > 5:
            return False
        return {
        "thoughts": "The aircraft is currently on heading 330, but ATC has instructed the aircraft to fly heading 174. To comply with the ATC instructions, I will issue the heading command to the autopilot. The current aircraft state suggests that it is already in straight and level flight, so no additional manual commands are needed.",
        "control interface": {
            "heading": int(input("Enter the desired heading in degrees: ")),
            "altitude": int(input("Enter the desired altitude in degrees: ")),
        },
        "complaints": "No complaints about the instructions or information provided."
        }

class JSONADMAlgorithm:
    def __init__(self):
        self.num_calls = 0
        self.MAX_CALLS = 5

    def get_decision(self, system_prompt, cur_prompt):
        self.num_calls += 1

        # Convert dicts to string prompt
        system_prompt_str = OpenRouterAgent.format_system_prompt(system_prompt)
        cur_prompt_str = json.dumps(cur_prompt, indent=2, ensure_ascii=False)

        if self.num_calls == 1:
            print("=" * 20 + " System Prompt " + "=" * 20)
            print(system_prompt_str)
        print("=" * 20 + " Current Prompt " + "=" * 20)
        print(cur_prompt_str)

        input("Continue?: \n")
        with open('response.txt', 'r') as file:
            model_response = file.read()

        print("=" * 20 + " Model Response " + "=" * 20)
        print(model_response)
        print("=" * 20 + " End Model Response " + "=" * 20)
        model_response = model_response[model_response.index("{"):model_response.rindex("}") + 1]  # Extract JSON part
        model_response = OpenRouterAgent.remove_extraneuous(model_response)

        dict_response = json.loads(model_response)
        if self.num_calls > self.MAX_CALLS:
            print(f"Warning: Maximum number of calls ({self.MAX_CALLS}) exceeded. Returning last response.")
            return False
        
        return dict_response


scenario = InstructionFollowing(
    ADM_algorithm=OpenRouterAgent(OpenRouterAgent.MISTRAL),
    case=InstructionFollowingCases.CLIMB_TO_ALTITUDE,
    render_mode="",
    custom_tracking_vars=[
        prp.propeller_rpm
    ]
)
scenario.start_environment()
scenario.run()
print(scenario.get_end_info())
scenario.plot_tracking_variables()

# Testing the JSON
response = """
# {
#   "thoughts": "The aircraft needs to turn left to heading 030 due to ATC instructions. The current heading is 330.0, which is 270 degrees away from the required heading. I will calculate the turn to reach the heading 030, ensuring the bank angle does not exceed 30 degrees.",
#   "control interface": {
#     "heading": 030.0,
#     "altitude": 6000.03295257315
#   },
#   "complaints": "No complaints, the information provided is sufficient for the task."
# }
# """
# print(OpenRouterAgent.test_get_decision(response))