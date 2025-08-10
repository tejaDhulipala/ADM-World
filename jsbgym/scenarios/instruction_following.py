import random
from enum import Enum
from typing import Dict

from ..control_system.control_interface_default import ControlInterfaceDefault
from .scenario_base import Scenario 
from ..simulation_interface import SimulationInterface
from .. import properties as prp
from ..aircraft import c172x
from ..control_system.control_interface_base import ControlInterfaceBase
from ..OpenRouterAgent import ADMAgent


class InstructionFollowingCases(Enum):
    """Enum for different instruction following cases."""
    FLY_HEADING = "fly heading"
    FLY_LONG_HEADING = "fly long way to heading"
    CLIMB_TO_ALTITUDE = "climb to altitude"
    LEAVING_ALTITUDE_FLY_HEADING = "leaving altitude fly heading"


class InstructionFollowing(Scenario):
    """The main purpose of this class is to test the end-to-end pipeline of creating a scenario. 
    This scenario tests the basic functionality of an ADM algorithm to follow Air Traffic Control
    radar vectors under assumed Instrument Flight Rules coming out of Northern Colorado Regional airport. Here are the types of
    instructions that can be given:
    1. Turn the shortest direction to a specified heading.
    "... fly heading 270 degrees"
    2. Turn the long direction to a specified heading.
    "... turn left to heading 270 degrees ..."
    3. Climb or descend to a specified altitude.
    "... climb to 5000 feet ..."
    4. Turn after a climb
    "... leaving 1700 fly heading 230 ..."
    Number 4 is the only case with sequential decision making, with multiple calls to ADM algorithm.

    :param Scenario: _description_
    :type Scenario: _type_
    """
    MAX_TIME_S = 600  # Maximum time in seconds for the scenario to run
    HARD_FLOOR_ALTITUDE_FT = 200  # The hard floor altitude for the scenario, in feet AGL
    HEADING_ERR_TOLERANCE = 10  # The maximum error in heading in degrees
    ALTITUDE_ERR_TOLERANCE = 100
    STEPS_WITHIN_TOLERANCE = 150


    def __init__(self, ADM_algorithm: ADMAgent, case: InstructionFollowingCases=None, render_mode=None, control_interface: ControlInterfaceBase=ControlInterfaceDefault(),
                 custom_tracking_vars: list = []):
        super().__init__(ADM_algorithm)
        # 40°28'01"N 105°01'11"W currespons to the departure leg of runway 33 at Northern Colorado Regional Airport
        self.initial_conditions = { 
            prp.initial_latitude_geod_deg: 40.46694,
            prp.initial_longitude_geoc_deg: -105.01972,
            prp.initial_altitude_agl_ft: 300,
            prp.initial_heading_deg: 330,
            prp.initial_terrain_altitude_ft: 5000
        }
        if case is None:
            case =  random.choice(list(InstructionFollowingCases))
        self.intial_instruction = {
            "control interface": {
            },
            "manual": {
            }
        }
        self.atc_command = ""
        self.tail_number = "N172EX"
        self.case = case
        self.desired_alt_agl = self.initial_conditions[prp.initial_altitude_agl_ft]
        self.desired_hdg = self.initial_conditions[prp.initial_heading_deg]
        
        match case:
            case InstructionFollowingCases.FLY_HEADING:
                new_heading = round(random.uniform(270, 30))
                self.atc_command = f"Skyhawk {self.tail_number}, fly heading {new_heading}"
                self.initial_conditions[prp.initial_altitude_agl_ft] = 1000
                self.desired_hdg = new_heading
                self.desired_alt_agl = 1000
            case InstructionFollowingCases.CLIMB_TO_ALTITUDE:
                new_altitude = 7000
                self.atc_command = f"Skyhawk {self.tail_number}, climb {new_altitude}"
                self.desired_hdg = self.initial_conditions[prp.initial_heading_deg]
                self.desired_alt_agl = new_altitude - self.initial_conditions[prp.initial_terrain_altitude_ft]
            case InstructionFollowingCases.FLY_LONG_HEADING:
                raise NotImplementedError
            case InstructionFollowingCases.LEAVING_ALTITUDE_FLY_HEADING:
                raise NotImplementedError
            case _:
                raise ValueError(f"Unknown case: {case}")
        
        self.sim = SimulationInterface(
            aircraft=c172x,
            initial_conditions=self.initial_conditions,
            control_agent_interaction_freq=5,
            render_mode=render_mode
        )

        self.control_interface = control_interface
        self.system_prompt = None
        self.prompts = []
        self._first_intruction_received = False
        self.end_info = {
            "airspeed violation": False,
            "altitude violation": False,
            "success": False,
            "time s": -1,
        }
        self.steps_within_tolerance = 0

        # tracking variables for plotting
        # Errors
        self.trjs_alt_agl = []
        self.trjs_alt_msl = []
        self.trjs_hdg = []
        # Speeds
        self.trjs_kias = []
        self.trjs_ground_speed = []
        # Time
        self.trjs_time = []
        # Custom vars
        self.trjs_custom_tracking_vars = {var: [] for var in custom_tracking_vars}

    def start_environment(self):
        """Starts the simulation environment."""
        obs = self.sim.initialize()
        self.control_interface.set_instruction(self.intial_instruction, self.sim)
        if self.case in [InstructionFollowingCases.FLY_HEADING, InstructionFollowingCases.FLY_LONG_HEADING, \
                         InstructionFollowingCases.CLIMB_TO_ALTITUDE]:
            callback = "There will be no callback for this scenario, you give one instruction."
        else:
            callback = "You will be called again when the airplane changes altitude by more than 500 feet, or heading by 45 degrees."
        self.system_prompt = {
            "role": "You are the decision-making system for an autonomous aircraft. You use the information" 
            "provided to you to give commands to an autoplilot system responsible for controlling the aircraft.",
            "control system": {
                "description":"The actual control of the aircraft is handled by a futuristic FAA certfied autopilot system capable"
                "of following a specifed heading and altitude.",
                "commands": {
                    "heading": "The autopilot turns the aircraft the short route to the magnetic heading, not exceeding 30 degrees of bank. Required.",
                    "altitude": "The autopilot climbs, descends, or maintains to the specified altitude msl. It will adjust the "
                    "throttle to always ensure safe, but not necesserily optimal (Vx/Vy), climb or descent rates. Required.",
                    "wings level": "Overrides heading command and rolls wings level. 0 is off, 1 is on. Optional.",
                }
            },
            "manual commands": {
                "description": "You can also give manual commands to the autopilot system. These commands will override the autopilot.",
                "commands": {
                    "throttle": "Set throttle value in RPM",
                    "mixture": "Set mixture value from 0 (full lean) to 1 (full rich). The autopilot sets a mixture to 0.8 by default.",
                }
            },
            "output format": "You must output valid YAML within tags called <YAML>. It should look similar to this:"
            "<YAML>"
            "reasoning: Your reasoning here, required"
            "control interface: # required"
            "    heading: # in degrees, 0-360, required"
            "    altitude: # in feet MSL, required"
            "    wings level: # 0 or 1, 0 is off, 1 is on, optional"
            "manual: # optional"
            "    throttle: # in RPM, optional"
            "    mixture: # from 0 (full lean) to 1 (full rich), optional"
            "complaints: # compaints about the actual scenario presented, such as lacking some data or contridictions, required"
            "</YAML>",
        }
        self.prompts.append({
            "initial scenario": f" You are at Northern Colorado Regional Airport, and have just departed runway 33 under IFR. Your aircraft," 
            f"{self.tail_number} is at {self.sim.get_property(prp.altitude_sl_ft)} feet MSL, or {self.sim.get_property(prp.altitude_agl_ft)} feet AGL," 
            f"and has heading {self.sim.get_property(prp.heading_deg)} degrees. You hear ATC say '{self.atc_command}'. Your current objective is to follow" 
            f"the Air Traffic Control instructions.",
            "current aircraft instruction": "straight and level",
            "aircraft state": {prop.description: obs[prop] for prop in obs},
            "callbacks": callback
        })
        print("System prompt:", self.system_prompt)
        print("Initial prompt:", self.prompts[0])

    def run(self):
        while not self.is_terminated() and not self.is_truncated():
            obs = self.sim.step(self.control_interface.action(self.sim))
            # Let it run for 10 seconds before getting the first instruction
            if self.sim.get_property(prp.sim_time_s) > 10 and not self._first_intruction_received:
                self.control_interface.set_instruction(self.ADM_algorithm.get_decision(self.system_prompt, self.prompts[0]), self.sim)
                self._first_intruction_received = True
            if self.callback_ADM_algorithm(self.intial_instruction):
                print("ADM algorithm callback received, setting new instruction.")
                self.control_interface.set_instruction(self.ADM_algorithm.get_decision(self.system_prompt, self.prompts[-1]), self.sim)
            # Add the current state to the tracking variables
            self.trjs_alt_agl.append(self.sim.get_property(prp.altitude_agl_ft))
            self.trjs_hdg.append(self.sim.get_property(prp.heading_deg))
            self.trjs_kias.append(self.sim.get_property(prp.cas_kts))
            self.trjs_ground_speed.append(self.sim.get_property(prp.groundspeed_fps) * 0.592484)  # Convert from fps to knots
            self.trjs_time.append(self.sim.get_property(prp.sim_time_s))
            self.trjs_alt_msl.append(self.sim.get_property(prp.altitude_sl_ft))
            for var in self.trjs_custom_tracking_vars:
                self.trjs_custom_tracking_vars[var].append(self.sim.get_property(var))
        self.end_info["time s"] = self.sim.get_property(prp.sim_time_s)
        self._close_scenario()

    def is_terminated(self):
        # Check airspeed violation
        if self.sim.get_property(prp.cas_kts) < self.sim.aircraft.Vx or self.sim.get_property(prp.cas_kts) > self.sim.aircraft.Vno:
            self.end_info["airspeed violation"] = True
            return True
        # Check altitude violation
        if self.sim.get_property(prp.altitude_agl_ft) < self.HARD_FLOOR_ALTITUDE_FT:
            self.end_info["altitude violation"] = True
            return True
        # Check if the aircraft has met the objective
        if (abs(self.sim.get_property(prp.heading_deg) - self.desired_hdg) < self.HEADING_ERR_TOLERANCE and
            abs(self.sim.get_property(prp.altitude_agl_ft) - self.desired_alt_agl) < self.ALTITUDE_ERR_TOLERANCE):
            self.steps_within_tolerance += 1
        if self.steps_within_tolerance >= self.STEPS_WITHIN_TOLERANCE:
            self.end_info["success"] = True
            self.end_info["time s"] = self.sim.get_property(prp.sim_time_s)
            return True
        return False

    def is_truncated(self):
        if self.sim.get_property(prp.sim_time_s) > self.MAX_TIME_S:
            return True
        return False

    def callback_ADM_algorithm(self, instruction: Dict) -> bool:
        if not self.case == InstructionFollowingCases.LEAVING_ALTITUDE_FLY_HEADING:
            return False

    def _close_scenario(self):
        self.sim.close()

    def get_end_info(self):
        return self.end_info
    
    def plot_tracking_variables(self):
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(3, 2, figsize=(15, 10))
        axs[0, 0].plot(self.trjs_time, self.trjs_alt_agl, label='AGL')
        axs[0, 0].legend()
        axs[0, 0].set_title('Altitude (ft AGL)')
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylabel('Altitude (ft AGL)')

        axs[0, 1].plot(self.trjs_time, self.trjs_hdg)
        axs[0, 1].set_title('Heading (degrees)')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Heading (degrees)')

        axs[1, 0].plot(self.trjs_time, self.trjs_kias, label='KIAS')
        axs[1, 0].plot(self.trjs_time, self.trjs_ground_speed, label='Ground Speed (knots)')
        axs[1, 0].legend()
        axs[1, 0].set_title('Airspeed (KIAS)')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Airspeed (KIAS)')

        axs[2, 1].plot(self.trjs_time, self.trjs_alt_msl, label='MSL')
        axs[2, 1].legend()
        axs[2, 1].set_title('Altitude (ft MSL)')
        axs[2, 1].set_xlabel('Time (s)')
        axs[2, 1].set_ylabel('Altitude (ft MSL)')

        for var in self.trjs_custom_tracking_vars:
            axs[1, 1].plot(self.trjs_time, self.trjs_custom_tracking_vars[var], label=var)
            axs[1, 1].set_title(f"{var}")
            axs[1, 1].set_ylabel(var)
            axs[1, 1].legend()
            axs[1, 1].grid(True, which='both')
        
        plt.show()

        

