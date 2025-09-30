
from copy import deepcopy
from math import pi
from typing import Dict

from ..simulation_interface import SimulationInterface
from ..aircraft import *
from .cases import *
from .. import properties as prp 
import random

class RPFlightControlEval:
    """
    RP stands for roll and pitch, which are the parameters given to the control system. The goal of the control
    system is to hold a certain roll and pitch
    """
    ACCEPTABLE_ROLL_ERR = 3
    ACCEPTABLE_FLIGHT_PATH_ERR = 3
    def __init__(self, aircraft: Aircraft=None, custom_tracking_vars:list[str]=[]) -> None:
        # --> Set-up trajectory data. Each of these is an array of trajectories for each eval run
        # Control surfaces
        self.trjs_aileron_pos_left = []
        self.trjs_aileron_pos_right = []
        self.trjs_aileron_fcs_cmd = []
        self.trjs_aileron_ap_cmd = []
        self.trjs_elevator_fcs_cmd = []
        self.trjs_elevator_ap_cmd = []
        self.trjs_elevator_trim = []
        self.trjs_rudder_fcs_cmd = []
        self.trjs_elevator_pos = []
        self.trjs_rudder_fcs_pos = []
        # Errors
        self.trjs_flight_path_error = []
        self.trjs_roll_error = []
        # Speeds
        self.trjs_kias = []
        self.trjs_ground_speed = []
        # Time
        self.trjs_time = []
        
        # The initial conditions and cases of each of the eval runs. 
        self.initial_conditions = [] # list of dictionaries
        self.cases = [] # list of tuples
        self.evals = [] # list of dictionaries
        self.sorted_indices = None
        self.trjs_custom_tracking_vars = {var: [] for var in custom_tracking_vars}

        self.aircraft = aircraft


    def run_single_eval(self, pitch_case: PitchCase, roll_case: RollCase, control_subsystem,
     render_mode=None, aircraft:Aircraft=c172x, as_case:AirSpeedCase=AirSpeedCase.AS_CRUISE) -> Dict:
        """Runs a single evaluation by taking in a control subsystem and cases for alt, heading, and max crosswind speed

        :param pitch_case: Which pitch case to simulate
        :type pitch_case: PitchCase
        :param roll_case: Which roll case to simulate
        :type roll_case: RollCase
        :param control_subsystem: This is an object that has an instance method called action(). action() should take in
        a SimulationInterface, then the desired pitch, then the desired roll. It should output of dictionary of 
        properties (keys) and values (values).
        :type control_subsystem: object
        :param render_mode: What, if at all, the test case should render. Options are human_fg, graph_fg, flightgear, human,
        graph, defaults to None
        :type render_mode: _type_, optional
        :param aircraft: Arcraft to simulate, defaults to c172x
        :type aircraft: Aircraft, optional
        :param as_case: What level of wind to use, defaults to AirSpeedCase.CLM
        :type as_case: AirSpeedCase, optional
        :raises AssertionError: _description_
        :raises AssertionError: _description_
        :raises AssertionError: _description_
        :raises AssertionError: _description_
        :return: conditions, evaluations. Conditions describes the precise initial conditions
        
        Evaluations is a dictionary of evaluations methods, which include:
        cur_eval = {
            "max alt overshoot": 0,
            "max hdg overshoot": 0,
            "max load factor": 0,
            "avg load factor": 0,
            "min airspeed": initial_airspeed_fps / KTS_TO_FT_PER_S,
            "max airspeed": -1,
            "avg airspeed": 0,
            "avg alt steady state error": 0,
            "avg hdg steady state error": 0,
            "time to first contact s": -1,
            "max man time mins": max_maneuver_time_mins,
            "max time mins": max_steps / interaction_freq / 60
        }
        :rtype: Dict
        """
        # --> Ensure correct formatting
        assert isinstance(pitch_case, PitchCase)
        assert isinstance(roll_case, RollCase)
        assert isinstance(as_case, AirSpeedCase)
        try:
            sim = None
            mock_control = deepcopy(control_subsystem)
            act = mock_control.action(sim, 10, 0) # SimulationInterface, pitch, roll
            assert type(act) == dict
        except AttributeError:
            pass
        except TypeError or AssertionError:
            raise AssertionError("The altitude or heading algorithm is not is the correct format")
        
        # Change self.aircraft
        self.aircraft = aircraft
        
        # --> Change variables based on the case
        start_pitch = 0
        start_roll = 0
        des_pitch = 0
        des_roll = 0
        start_airspeed = 0
        match pitch_case:
            case PitchCase.PITCH_LVL:
                start_pitch = 0
                des_pitch = 0
            case PitchCase.PITCH_RCVR_10:
                start_pitch = random.uniform(3, 10)
                des_pitch = 0
            case PitchCase.PITCH_RCVR_M_10:
                start_pitch = -random.uniform(3, 10)
                des_pitch = 0
            case PitchCase.PITCH_RCVR_STALL:
                start_pitch = random.uniform(15, 50)
                des_pitch = 0
            case PitchCase.PITCH_RCVR_DSC:
                start_pitch = -random.uniform(15, 50)
                des_pitch = 0
            case PitchCase.PITCH_CRSE_CLMB:
                start_pitch = 0
                des_pitch = 3
                # des_pitch = random.uniform(2, 10)
            case PitchCase.PITCH_HIGH_CLMB:
                start_pitch = 0
                des_pitch = random.uniform(10, 45)
            case _:
                raise AssertionError("Not a valid pitch case")
        
        match roll_case:
            case RollCase.ROLL_LVL:
                start_roll = random.uniform(-3, 3)
                des_roll = 0
            case RollCase.ROLL_RCVR_30:
                start_roll = random.uniform(5, 30) * random.choice([-1, 1])
                des_roll = 0
            case RollCase.ROLL_RCVR_60:
                start_roll = random.uniform(30, 60) * random.choice([-1, 1])
                des_roll = 0
            case RollCase.ROLL_30:
                start_roll = 0
                des_roll = random.uniform(10, 30) * random.choice([-1, 1])
            case RollCase.ROLL_45:
                start_roll = 0
                des_roll = random.uniform(30, 45) * random.choice([-1, 1])
            case _:
                raise AssertionError("Not a valid roll case")
        
        match as_case:
            case AirSpeedCase.AS_CRUISE:
                start_airspeed = aircraft.cruise_speed_kts
            case AirSpeedCase.AS_L_VNO:
                start_airspeed = random.uniform(aircraft.Vno-10,aircraft.Vno) 
            case AirSpeedCase.AS_VNO_VNE:
                start_airspeed = random.uniform(aircraft.Vno, aircraft.Vne)
            case AirSpeedCase.AS_P_VNE:
                start_airspeed = random.uniform(aircraft.Vne, aircraft.Vne + 30) # arbitrary max
            case AirSpeedCase.AS_S_CRUISE:
                start_airspeed = random.uniform(70, 90)
            case AirSpeedCase.AS_APPROACH:
                start_airspeed = random.uniform(50, 70)
            case AirSpeedCase.AS_STALL:
                start_airspeed = random.uniform(aircraft.Vs0-5, aircraft.Vs1 + 10)
            case _:
                raise AssertionError("Not a valid airspeed case")

        
        initial_alt = 5000
        initial_hdg = 0
        # Wind direction is in the middle of the turn. The wind direction for jsbsim in in radians, where north is 0 and increases 
        # counterclockwise to 2pi
        initial_airspeed_fps = start_airspeed * KTS_TO_FT_PER_S
        print(initial_airspeed_fps)
        initial_conditions = {
            prp.wind_heading: 0,
            prp.wind_speed: 0,
            prp.initial_altitude_msl_ft: initial_alt,
            prp.initial_heading_deg: initial_hdg,
            prp.initial_u_fps: initial_airspeed_fps,
            prp.initial_pitch_deg: start_pitch,
            prp.initial_roll_deg: start_roll
        }
        interaction_freq = 5

        # Calculate the steps of the simulation so it doesn't render more than it needs to & determine steps to consider steady state
        max_maneuver_time_mins = 0.25
        max_man_steps = max_maneuver_time_mins * 60 * interaction_freq
        max_steps = max_man_steps + 90 * interaction_freq # 90 seconds of steady state flight 
        print(f"Running simulation for {max_steps} steps")

        # Set-up eval metrics dictionary
        cur_eval = {
            "max pitch overshoot": 0,
            "max roll overshoot": 0,
            "max load factor": 0,
            "avg load factor": 0,
            "min airspeed": initial_airspeed_fps / KTS_TO_FT_PER_S,
            "max airspeed": -1,
            "avg airspeed": 0,
            "avg flight path ss error": 0,
            "avg roll ss error": 0,
            "time to first contact s": 0,
            "max man time mins": max_maneuver_time_mins,
            "max time mins": max_steps / interaction_freq / 60
        }

        # --> Set-up trajectory data
        # Control surfaces
        trj_aileron_pos_left = []
        trj_aileron_pos_right = []
        trj_aileron_fcs_cmd = []
        trj_aileron_ap_cmd = []
        trj_elevator_fcs_cmd = []
        trj_elevator_ap_cmd = []
        trj_elevator_trim = []
        trj_rudder_fcs_cmd = []
        trj_elevator_pos = []
        trj_rudder_fcs_pos = []
        # Errors
        trj_pitch_error = []
        trj_roll_error = []
        # Speeds
        trj_kias = []
        trj_ground_speed = []
        # Time
        trj_time = []
        # custom tracking vars
        trj_custom_tracking_vars = {var: [] for var in self.trjs_custom_tracking_vars}
        
        # --> Run sim
        sim = SimulationInterface(initial_conditions=initial_conditions, aircraft=aircraft, render_mode=render_mode,
        control_agent_interaction_freq=interaction_freq) # render mode can be None, flightgear, human, graph, human_fg, or graph_fg
        sim.initialize()

        num_steps = 0
        while num_steps < max_steps:
            # --> Execute actions
            actions = control_subsystem.action(sim, des_pitch, des_roll)
            obs = sim.step(actions)
            
            # --> Edit evaluations
            pitch_error = abs(des_pitch - sim.get_property(prp.pitch_rad) / pi * 180)
            roll_error = abs(des_roll - sim.get_property(prp.roll_rad) / pi * 180)
            
            # Max pitch overshoot
            cur_eval["max pitch overshoot"] = max(cur_eval["max pitch overshoot"], pitch_error)
            # Max roll overshoot
            cur_eval["max roll overshoot"] = max(cur_eval["max roll overshoot"], roll_error)
            # Max load factor
            cur_eval["max load factor"] = max(cur_eval["max load factor"], abs(sim.get_property(prp.load_factor)))
            # Avg load factor
            cur_eval["avg load factor"] += abs(sim.get_property(prp.load_factor))
            # Min airspeed
            cur_eval["min airspeed"] = min(cur_eval["min airspeed"], sim.get_property(prp.cas_kts))
            # Max airspeed
            cur_eval["max airspeed"] = max(cur_eval["max airspeed"], sim.get_property(prp.cas_kts))
            # Avg airspeed
            cur_eval["avg airspeed"] += sim.get_property(prp.cas_kts)
            # avg flight path ss error
            if num_steps > max_man_steps:
                cur_eval["avg flight path ss error"] += pitch_error
            # avg roll ss error
            if num_steps > max_man_steps:
                cur_eval["avg roll ss error"] += roll_error
            # Time to first contact
            if pitch_error < self.ACCEPTABLE_FLIGHT_PATH_ERR and roll_error < self.ACCEPTABLE_ROLL_ERR and not cur_eval["time to first contact s"]:
                cur_eval["time to first contact s"] = num_steps / interaction_freq
                
            # --> Update Trajectory Data
            # Control surfaces
            trj_aileron_pos_left.append(sim.get_property(prp.aileron_left))
            trj_aileron_pos_right.append(sim.get_property(prp.aileron_right))
            trj_aileron_fcs_cmd.append(sim.get_property(prp.aileron_cmd))
            trj_aileron_ap_cmd.append(sim.get_property("ap/aileron_cmd"))

            trj_elevator_fcs_cmd.append(sim.get_property(prp.elevator_cmd))
            trj_elevator_ap_cmd.append(sim.get_property("ap/elevator_cmd"))
            trj_elevator_trim.append(sim.get_property("fcs/pitch-trim-cmd-norm"))
            trj_rudder_fcs_cmd.append(sim.get_property(prp.rudder_cmd))
            trj_rudder_fcs_pos.append(sim.get_property(prp.rudder))
            trj_elevator_pos.append(sim.get_property(prp.elevator_rad))
            # Errors
            trj_pitch_error.append(pitch_error if sim.get_property(prp.pitch_rad) > des_pitch / 180 * pi else -pitch_error)
            trj_roll_error.append(roll_error if sim.get_property(prp.roll_rad) > des_roll / 180 * pi / 180 * pi else -roll_error)
            # Speeds
            trj_kias.append(sim.get_property(prp.cas_kts))
            trj_ground_speed.append(sim.get_property(prp.groundspeed_fps) / KTS_TO_FT_PER_S)
            # Time
            trj_time.append(sim.get_property(prp.sim_time_s))
            for var in self.trjs_custom_tracking_vars:
                trj_custom_tracking_vars[var].append(sim.get_property(var))

            # increment steps
            num_steps += 1
        sim.close()

        # Return the correct evaluation and exact conditions
        exact_conditions = {prop.name: initial_conditions[prop] for prop in initial_conditions}
        exact_conditions = exact_conditions | {"start pitch": start_pitch,
        "des_pitch": des_pitch,
        "start_roll": start_roll,
        "des_roll": des_roll}

        cur_eval["avg airspeed"] /= num_steps
        cur_eval["avg load factor"] /= num_steps
        cur_eval["avg flight path ss error"] /= num_steps
        cur_eval["avg roll ss error"] /= num_steps

        # Add the trajectory information to the instance
        self.trjs_aileron_pos_left.append(trj_aileron_pos_left)
        self.trjs_aileron_pos_right.append(trj_aileron_pos_right)
        self.trjs_aileron_fcs_cmd.append(trj_aileron_fcs_cmd)
        self.trjs_aileron_ap_cmd.append(trj_aileron_ap_cmd)
        self.trjs_elevator_fcs_cmd.append(trj_elevator_fcs_cmd)
        self.trjs_elevator_ap_cmd.append(trj_elevator_ap_cmd)
        self.trjs_elevator_trim.append(trj_elevator_trim)
        self.trjs_elevator_pos.append(trj_elevator_pos)
        self.trjs_rudder_fcs_cmd.append(trj_rudder_fcs_cmd)
        self.trjs_rudder_fcs_pos.append(trj_rudder_fcs_pos)
        self.trjs_flight_path_error.append(trj_pitch_error)
        self.trjs_roll_error.append(trj_roll_error)
        self.trjs_kias.append(trj_kias)
        self.trjs_ground_speed.append(trj_ground_speed)
        self.trjs_time.append(trj_time)
        self.initial_conditions.append(exact_conditions)
        self.evals.append(cur_eval)
        self.cases.append((pitch_case, roll_case, as_case))
        for var in trj_custom_tracking_vars:
            self.trjs_custom_tracking_vars[var].append(trj_custom_tracking_vars[var])

        return exact_conditions, cur_eval
    
    def run_specific_eval(self, start_flight_path: float, des_flight_path: float, start_roll: float, 
    des_roll: float, control_subsystem, render_mode=None, aircraft:Aircraft=c172x, 
    start_airspeed=92) -> Dict:
        """
        Evaluations is a dictionary of evaluations methods, which include:
        cur_eval = {
            "max alt overshoot": 0,
            "max hdg overshoot": 0,
            "max load factor": 0,
            "avg load factor": 0,
            "min airspeed": initial_airspeed_fps / KTS_TO_FT_PER_S,
            "max airspeed": -1,
            "avg airspeed": 0,
            "avg alt steady state error": 0,
            "avg hdg steady state error": 0,
            "time to first contact s": -1,
            "max man time mins": max_maneuver_time_mins,
            "max time mins": max_steps / interaction_freq / 60
        }
        :rtype: Dict
        """
        # --> Ensure correct formatting
        try:
            sim = None
            mock_control = deepcopy(control_subsystem)
            act = mock_control.action(sim, 10, 0) # SimulationInterface, pitch, roll
            assert type(act) == dict
        except AttributeError:
            pass
        except TypeError or AssertionError:
            raise AssertionError("The altitude or heading algorithm is not is the correct format")
        
        # Change self.aircraft
        self.aircraft = aircraft

        initial_alt = 5000
        initial_hdg = 0
        # Wind direction is in the middle of the turn. The wind direction for jsbsim in in radians, where north is 0 and increases 
        # counterclockwise to 2pi
        initial_airspeed_fps = start_airspeed * KTS_TO_FT_PER_S
        print(initial_airspeed_fps)
        initial_conditions = {
            prp.wind_heading: 0,
            prp.wind_speed: 0,
            prp.initial_altitude_msl_ft: initial_alt,
            prp.initial_heading_deg: initial_hdg,
            prp.initial_u_fps: initial_airspeed_fps,
            prp.initial_flight_path: start_flight_path,
            prp.initial_roll_deg: start_roll
        }
        interaction_freq = 5

        # Calculate the steps of the simulation so it doesn't render more than it needs to & determine steps to consider steady state
        max_maneuver_time_mins = 0.25
        max_man_steps = max_maneuver_time_mins * 60 * interaction_freq
        max_steps = max_man_steps + 90 * interaction_freq # 90 seconds of steady state flight 
        print(f"Running simulation for {max_steps} steps")

        # Set-up eval metrics dictionary
        cur_eval = {
            "max pitch overshoot": 0,
            "max roll overshoot": 0,
            "max load factor": 0,
            "avg load factor": 0,
            "min airspeed": initial_airspeed_fps / KTS_TO_FT_PER_S,
            "max airspeed": -1,
            "avg airspeed": 0,
            "avg flight path ss error": 0,
            "avg roll ss error": 0,
            "time to first contact s": 0,
            "max man time mins": max_maneuver_time_mins,
            "max time mins": max_steps / interaction_freq / 60
        }

        # --> Set-up trajectory data
        # Control surfaces
        trj_aileron_pos_left = []
        trj_aileron_pos_right = []
        trj_aileron_fcs_cmd = []
        trj_aileron_ap_cmd = []
        trj_elevator_fcs_cmd = []
        trj_elevator_ap_cmd = []
        trj_elevator_trim = []
        trj_rudder_fcs_cmd = []
        trj_elevator_pos = []
        trj_rudder_fcs_pos = []
        # Errors
        trj_flight_path_error = []
        trj_roll_error = []
        # Speeds
        trj_kias = []
        trj_ground_speed = []
        # Time
        trj_time = []
        # custom tracking vars
        trj_custom_tracking_vars = {var: [] for var in self.trjs_custom_tracking_vars}
        
        # --> Run sim
        sim = SimulationInterface(initial_conditions=initial_conditions, aircraft=aircraft, render_mode=render_mode,
        control_agent_interaction_freq=interaction_freq) # render mode can be None, flightgear, human, graph, human_fg, or graph_fg
        sim.initialize()

        num_steps = 0
        while num_steps < max_steps:
            # --> Execute actions
            actions = control_subsystem.action(sim, des_flight_path, des_roll)
            obs = sim.step(actions)
            
            # --> Edit evaluations
            flight_path_error = abs(des_flight_path - sim.get_property(prp.flight_path))
            roll_error = abs(des_roll - sim.get_property(prp.roll_rad) / pi * 180)
            
            # Max pitch overshoot
            cur_eval["max pitch overshoot"] = max(cur_eval["max pitch overshoot"], flight_path_error)
            # Max roll overshoot
            cur_eval["max roll overshoot"] = max(cur_eval["max roll overshoot"], roll_error)
            # Max load factor
            cur_eval["max load factor"] = max(cur_eval["max load factor"], abs(sim.get_property(prp.load_factor)))
            # Avg load factor
            cur_eval["avg load factor"] += abs(sim.get_property(prp.load_factor))
            # Min airspeed
            cur_eval["min airspeed"] = min(cur_eval["min airspeed"], sim.get_property(prp.cas_kts))
            # Max airspeed
            cur_eval["max airspeed"] = max(cur_eval["max airspeed"], sim.get_property(prp.cas_kts))
            # Avg airspeed
            cur_eval["avg airspeed"] += sim.get_property(prp.cas_kts)
            # avg flight path ss error
            if num_steps > max_man_steps:
                cur_eval["avg flight path ss error"] += flight_path_error
            # avg roll ss error
            if num_steps > max_man_steps:
                cur_eval["avg roll ss error"] += roll_error
            # Time to first contact
            if flight_path_error < self.ACCEPTABLE_FLIGHT_PATH_ERR and roll_error < self.ACCEPTABLE_ROLL_ERR and not cur_eval["time to first contact s"]:
                cur_eval["time to first contact s"] = num_steps / interaction_freq
                
            # --> Update Trajectory Data
            # Control surfaces
            trj_aileron_pos_left.append(sim.get_property(prp.aileron_left))
            trj_aileron_pos_right.append(sim.get_property(prp.aileron_right))
            trj_aileron_fcs_cmd.append(sim.get_property(prp.aileron_cmd))
            trj_aileron_ap_cmd.append(sim.get_property("ap/aileron_cmd"))

            trj_elevator_fcs_cmd.append(sim.get_property(prp.elevator_cmd))
            trj_elevator_ap_cmd.append(sim.get_property("ap/elevator_cmd"))
            trj_elevator_trim.append(sim.get_property("fcs/pitch-trim-cmd-norm"))
            trj_rudder_fcs_cmd.append(sim.get_property(prp.rudder_cmd))
            trj_rudder_fcs_pos.append(sim.get_property(prp.rudder))
            trj_elevator_pos.append(sim.get_property(prp.elevator_rad))
            # Errors
            trj_flight_path_error.append(flight_path_error if sim.get_property(prp.flight_path) > des_flight_path else -flight_path_error)
            trj_roll_error.append(roll_error if sim.get_property(prp.roll_rad) > des_roll / 180 * pi / 180 * pi else -roll_error)
            # Speeds
            trj_kias.append(sim.get_property(prp.cas_kts))
            trj_ground_speed.append(sim.get_property(prp.groundspeed_fps) / KTS_TO_FT_PER_S)
            # Time
            trj_time.append(sim.get_property(prp.sim_time_s))
            for var in self.trjs_custom_tracking_vars:
                trj_custom_tracking_vars[var].append(sim.get_property(var))

            # increment steps
            num_steps += 1
        sim.close()

        # Return the correct evaluation and exact conditions
        exact_conditions = {prop.name: initial_conditions[prop] for prop in initial_conditions}
        exact_conditions = exact_conditions | {"start pitch": start_flight_path,
        "des_pitch": des_flight_path,
        "start_roll": start_roll,
        "des_roll": des_roll}

        cur_eval["avg airspeed"] /= num_steps
        cur_eval["avg load factor"] /= num_steps
        cur_eval["avg flight path ss error"] /= num_steps
        cur_eval["avg roll ss error"] /= num_steps

        # Add the trajectory information to the instance
        self.trjs_aileron_pos_left.append(trj_aileron_pos_left)
        self.trjs_aileron_pos_right.append(trj_aileron_pos_right)
        self.trjs_aileron_fcs_cmd.append(trj_aileron_fcs_cmd)
        self.trjs_aileron_ap_cmd.append(trj_aileron_ap_cmd)
        self.trjs_elevator_fcs_cmd.append(trj_elevator_fcs_cmd)
        self.trjs_elevator_ap_cmd.append(trj_elevator_ap_cmd)
        self.trjs_elevator_trim.append(trj_elevator_trim)
        self.trjs_elevator_pos.append(trj_elevator_pos)
        self.trjs_rudder_fcs_cmd.append(trj_rudder_fcs_cmd)
        self.trjs_rudder_fcs_pos.append(trj_rudder_fcs_pos)
        self.trjs_flight_path_error.append(trj_flight_path_error)
        self.trjs_roll_error.append(trj_roll_error)
        self.trjs_kias.append(trj_kias)
        self.trjs_ground_speed.append(trj_ground_speed)
        self.trjs_time.append(trj_time)
        self.initial_conditions.append(exact_conditions)
        self.evals.append(cur_eval)
        self.cases.append((None, None, None))
        for var in trj_custom_tracking_vars:
            self.trjs_custom_tracking_vars[var].append(trj_custom_tracking_vars[var])

        return exact_conditions, cur_eval
    
    def batch_test(self, pitch_cases: list[PitchCase], roll_cases: list[PitchCase], as_cases: list[AirSpeedCase], control_subsystem
    ,num_trials: int, aircraft:Aircraft=c172x):
        """Runs a complets batch of tests and saves all data to the instance varibales of the class. Does all
        combinations of the cases given. Prints out initial conditions and evaluations for each case. 
        
        :param pitch_cases: List of altitude cases 
        :type pitch_cases: list[PitchCase]
        :param roll_cases: List of heading cases
        :type roll_cases: list[PitchCase]
        :param as_cases: list of wind cases
        :type as_cases: list[AirSpeedCase]
        :param control_subsystem: An object that has a method action(SimulationInterface, des_alt, des_hdg)
        :type control_subsystem: object
        :param num_trials: The number of trials to run for each case
        :type num_trials: int
        :param aircraft: Aircraft type of type Aircraft, defaults to c172x
        :type aircraft: Aircraft, optional
        """

        for pitch_case in pitch_cases:
            for roll_case in roll_cases:
                for as_case in as_cases:
                    print(f"=== CASE PITCH: {pitch_case} ROLL: {roll_case} ALTITUDE: {as_case} ===")
                    for i in range(num_trials):
                        print(f"--> EXAMPLE {i}")
                        ics, evals = self.run_single_eval(pitch_case, roll_case, deepcopy(control_subsystem), aircraft=aircraft, as_case=as_case)
                        print(f"Average steady state error:{evals["avg flight path ss error"]}")
                        print(f"Time to first conatact: {evals["time to first contact s"]}") 
    
    def sort_evals(self) -> tuple[list[int], list[tuple]]:
        """Returns a list of the index of individual evalutaions sorted by from worst to best. It also changes the values of self.sorted_evals
        to the sorted_evals.
        The evaluation criteria is:
        1. avg steady state flight path error rounded to the nearest 3 degrees
        2. max pitch overshoot to the nearest 3
        3. time to first contact greater than the expected time (true / false)
        4. max load factor to the nearest 0.5

        :return: indices, criteria. The inidices are The list of indexes of evaluations in self.evals. The criteria is a list
        of tuples describing how this function evaluated each eval as stated above. 
        :rtype: list[int]
        """
        def round_to_nearest(value, base):
            return round(value / base) * base

        def eval_sort_key(eval):
            # Criterion 1: Avg alt steady-state error (rounded to nearest 50)
            avg_pitch_error_rounded = round_to_nearest(eval["avg flight path ss error"], 3)

            # Criterion 2: Max alt overshoot (rounded to nearest 50)
            max_pitch_overshoot_rounded = round_to_nearest(eval["max pitch overshoot"], 3)

            # Criterion 3: Late contact time (True if time > expected)
            buffer_time_s = 5
            late_contact = eval["time to first contact s"] > (eval["max man time mins"] / 60 + buffer_time_s)

            # Criterion 4: Max load factor (rounded to nearest 0.5)
            max_load_factor_rounded = round_to_nearest(eval["max load factor"], 0.5)

            return (
                avg_pitch_error_rounded,
                max_pitch_overshoot_rounded,
                int(late_contact),                  # 0 = on-time or earlier, 1 = late
                max_load_factor_rounded
            )

        indexed_evals = list(enumerate(self.evals))
        indexed_evals.sort(key=lambda pair: eval_sort_key(pair[1]))
        indexed_evals.reverse() # Reverse from best to worst to worst to best

        self.sorted_indices = [idx for idx, _ in indexed_evals]
        criteria = [eval_sort_key(pair[1]) for pair in indexed_evals]
        return self.sorted_indices, criteria

    def plot_eval(self, index=-1):
        """
        Given the index, it uses the trajectory data to create three matplotlib charts. The X-axis of each chart will be the 
        time series from self.trjs_time. Then, data will be plotted on the y axis. The first chart will have all control commands
        and deflections, from aileron pos left to rudder_fcs_pos. The second chart will have the heading and altitude error. The 
        third chart will have the ground speed, and indicated airspeed. Each chart will have a legend and appropriate title. 

        :param index: Which evaluation, indexed at zero, you want to plot, defaults to -1
        :type index: int, optional
        """
        import matplotlib.pyplot as plt
        from matplotlib.ticker import MultipleLocator

        # Get the correct index
        idx = index if index >= 0 else -1

        time = self.trjs_time[idx]
        fig, axs = plt.subplots(5, 1, figsize=(14, 16), sharex=True)

        # First chart: All control commands and deflections
        # axs[0].plot(time, self.trjs_aileron_pos_left[idx], label="Aileron Pos Left")
        # axs[0].plot(time, self.trjs_aileron_pos_right[idx], label="Aileron Pos Right")
        # axs[0].plot(time, self.trjs_aileron_fcs_cmd[idx], label="Aileron FCS Cmd")
        # axs[0].plot(time, self.trjs_aileron_ap_cmd[idx], label="Aileron AP Cmd")
        axs[0].plot(time, self.trjs_elevator_fcs_cmd[idx], label="Elevator FCS Cmd")
        # axs[0].plot(time, self.trjs_elevator_ap_cmd[idx], label="Elevator AP Cmd")
        axs[0].plot(time, self.trjs_elevator_pos[idx], label="Elevator Pos Rad")
        axs[0].plot(time, self.trjs_elevator_trim[idx], label="Elevator Trim")
        # axs[0].plot(time, self.trjs_rudder_fcs_cmd[idx], label="Rudder FCS Cmd")
        # axs[0].plot(time, self.trjs_rudder_fcs_pos[idx], label="Rudder FCS Pos")
        axs[0].set_title("All Control Commands and Deflections")
        axs[0].set_ylabel("Normalized Value")
        axs[0].legend()
        axs[0].grid(True, which='both')

        # Second chart: flight path error
        axs[1].plot(time, self.trjs_flight_path_error[idx], label="flight path error deg")
        axs[1].set_title("flight path error")
        axs[1].set_ylabel("Error")
        axs[1].legend()
        axs[1].axvline(x=self.evals[index]["max man time mins"]*60)
        axs[1].grid(True, which='both')

        # Third chart: roll error
        axs[2].plot(time, self.trjs_roll_error[idx], label="Roll Error [deg]")
        axs[2].set_title("Roll Error")
        axs[2].set_ylabel("Error")
        axs[2].legend()
        axs[2].grid(True, which='both')
        axs[2].axvline(x=self.evals[index]["max man time mins"]*60)
        y_minor_spacing = 10
        minor_locator = MultipleLocator(y_minor_spacing)
        axs[2].yaxis.set_minor_locator(minor_locator)

        # Fourth chart: ground speed and indicated airspeed
        axs[3].plot(time, self.trjs_ground_speed[idx], label="Ground Speed [kts]")
        axs[3].plot(time, self.trjs_kias[idx], label="Indicated Airspeed [kts]")
        axs[3].set_title("Speeds")
        axs[3].set_xlabel("Time [s]")
        axs[3].set_ylabel("Speed [kts]")
        axs[3].legend()
        axs[3].grid(True, which='both')

        # Custom tracking vars
        for var in self.trjs_custom_tracking_vars:
            axs[4].plot(time, self.trjs_custom_tracking_vars[var][idx], label=var)
            axs[4].set_title(f"{var}")
            axs[4].set_ylabel(var)
            axs[4].legend()
            axs[4].grid(True, which='both')

        plt.tight_layout()
        plt.minorticks_on()
        plt.show()