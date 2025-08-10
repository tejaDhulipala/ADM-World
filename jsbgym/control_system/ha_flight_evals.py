from copy import deepcopy
from math import pi
from typing import Dict

from ..aircraft import *
from ..simulation_interface import SimulationInterface
from .. import properties as prp
from .cases import *
import random

class HAFlightControlEval:
    """
    HA stands for heading and altitude, which are the parameters given to the control system. The purpose of this class is to test and produce evaluation metrics of the ability of the airplane to do any combination of the four basic flight
    maneuvers: climbs, descents, turns, and straight & level in the normal (not slow flight) control regime. 
    """
    ACCEPTABLE_HEADING_ERR = 7.5
    ACCEPTABLE_ALTITUDE_ERR = 50
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
        self.trjs_alt_error = []
        self.trjs_hdg_error = []
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


    def run_single_eval(self, alt_case: AltCase, hdg_case: HdgCase, control_subsystem,
     render_mode=None, aircraft:Aircraft=c172x, wind_case:WindCase=WindCase.CLM) -> Dict:
        """Runs a single evaluation by taking in a control subsystem and cases for alt, heading, and max crosswind speed

        :param alt_case: Which altitude range to simulate
        :type alt_case: AltCase
        :param hdg_case: Which heading change case to simulate
        :type hdg_case: HdgCase
        :param subsystem_agregator: This is an object that has an instance method called action(). action() should take in
        a SimulationInterface, then the desired altitude, then the desired heading. It should output of dictionary of 
        properties (keys) and values (values).
        :type subsystem_agregator: object
        :param render_mode: What, if at all, the test case should render. Options are human_fg, graph_fg, flightgear, human,
        graph, defaults to None
        :type render_mode: _type_, optional
        :param aircraft: Arcraft to simulate, defaults to c172x
        :type aircraft: Aircraft, optional
        :param wind_case: What level of wind to use, defaults to WindCase.CLM
        :type wind_case: WindCase, optional
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
        assert isinstance(alt_case, AltCase)
        assert isinstance(hdg_case, HdgCase)
        assert isinstance(wind_case, WindCase)
        try:
            sim = None
            mock_control = deepcopy(control_subsystem)
            act = mock_control.action(sim, 5000, 90)
            assert type(act) == dict
        except AttributeError:
            pass
        except TypeError or AssertionError:
            raise AssertionError("The altitude or heading algorithm is not is the correct format")
        
        # Change self.aircraft
        self.aircraft = aircraft
        
        # --> Change variables based on the case
        alt_change = 0
        hdg_change = 0
        wind_speed = 0
        match alt_case:
            case AltCase.ALT_HOLD:
                alt_change = 0
            case AltCase.CLB_L_200:
                alt_change = random.uniform(20, 200)
            case AltCase.CLB_200_500:
                alt_change = random.uniform(250, 550)
            case AltCase.CLB_G_500:
                alt_change = random.uniform(700, 3000)
            case AltCase.DSC_L_200:
                alt_change = -random.uniform(20, 200)
            case AltCase.DSC_200_500:
                alt_change = -random.uniform(250, 550)
            case AltCase.DSC_G_500:
                alt_change = -random.uniform(700, 3000)
            case _:
                raise AssertionError("Not a valid altitude case")
        
        match hdg_case:
            case HdgCase.HDG_HOLD:
                hdg_change = 0
            case HdgCase.HDG_L_45:
                hdg_change = random.uniform(5, 45) * random.choice([-1, 1])
            case HdgCase.HDG_45_90:
                hdg_change = random.uniform(45, 90) * random.choice([-1, 1])
            case HdgCase.HDG_90_180:
                hdg_change = random.uniform(90, 180) * random.choice([-1, 1])
            case _:
                raise AssertionError("Not a valid heading case")
        
        match wind_case:
            case WindCase.CLM:
                wind_speed = 0
            case WindCase.WND_L_5:
                wind_speed = random.uniform(1, 5)
            case WindCase.WND_5_10:
                wind_speed = random.uniform(6, 10)
            case WindCase.WND_10_15:
                wind_speed = random.uniform(11, 15)
            case WindCase.WND_G_15:
                wind_speed = max(random.normalvariate(25, 4), 0)
            case _:
                raise AssertionError("Not a valid wind case")

        
        initial_alt = 5000
        initial_hdg = 0
        des_alt = initial_alt + alt_change
        des_hdg = initial_hdg + hdg_change
        # Wind direction is in the middle of the turn. The wind direction for jsbsim in in radians, where north is 0 and increases 
        # counterclockwise to 2pi
        wind_direction = -(initial_hdg + hdg_change / 2) / 180 * pi + 2 * pi
        initial_airspeed_fps = aircraft.get_cruise_speed_fps()
        initial_conditions = {
            prp.initial_wind_heading: wind_direction,
            prp.initial_wind_speed: wind_speed,
            prp.initial_altitude_msl_ft: initial_alt,
            prp.initial_heading_deg: initial_hdg,
            prp.initial_u_fps: initial_airspeed_fps
        }
        interaction_freq = 5

        # Calculate the steps of the simulation so it doesn't render more than it needs to & determine steps to consider steady state
        max_maneuver_time_mins = max(abs(alt_change) / 500, abs(hdg_change) / 180) * 1.5
        max_man_steps = max_maneuver_time_mins * 60 * interaction_freq
        max_steps = max_man_steps + 90 * interaction_freq # 90 seconds of steady state flight 
        print(f"Running simulation for {max_steps} steps")

        # Set-up eval metrics dictionary
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
        trj_alt_error = []
        trj_hdg_error = []
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
            actions = control_subsystem.action(sim, des_alt, des_hdg)
            obs = sim.step(actions)
            
            # --> Edit evaluations
            # Edit max alt overshoot
            alt_error = abs(des_alt - sim.get_property(prp.altitude_sl_ft))
            hdg_error = min(abs(des_hdg - sim.get_property(prp.heading_deg)), 360 - abs(des_hdg - sim.get_property(prp.heading_deg)))
            if alt_change < 0 and sim.get_property(prp.altitude_sl_ft) < des_alt:
                cur_eval["max alt overshoot"] = max(cur_eval["max alt overshoot"], alt_error)
            elif alt_change > 0 and sim.get_property(prp.altitude_sl_ft) > des_alt:
                cur_eval["max alt overshoot"] = max(cur_eval["max alt overshoot"], alt_error)
            # Edit max hdg overshoot
            if hdg_change < 0 and sim.get_property(prp.heading_deg) < des_hdg:
                cur_eval["max hdg overshoot"] = max(cur_eval["max hdg overshoot"], hdg_error)
            elif hdg_change > 0 and sim.get_property(prp.heading_deg) > des_hdg:
                cur_eval["max hdg overshoot"] = max(cur_eval["max hdg overshoot"], hdg_error)
            # Max load factor
            cur_eval["max load factor"] = max(cur_eval["max load factor"], abs(sim.get_property(prp.load_factor)))
            # Avg load factor. I'm not actually calculating the avg yet for simplicity
            cur_eval["avg load factor"] += abs(sim.get_property(prp.load_factor))
            # Min airspeed
            cur_eval["min airspeed"] = min(cur_eval["min airspeed"], sim.get_property(prp.cas_kts))
            # max airspeed
            cur_eval["max airspeed"] = max(cur_eval["max airspeed"], sim.get_property(prp.cas_kts))
            # avg airspeed, but sum for now
            cur_eval["avg airspeed"] += sim.get_property(prp.cas_kts)
            # avg alt steady state error
            if num_steps > max_man_steps:
                cur_eval["avg alt steady state error"] += alt_error
            # avg hdg steady state error
            if num_steps > max_man_steps:
                cur_eval["avg hdg steady state error"] += hdg_error
            # Time to first contact
            if alt_error < self.ACCEPTABLE_ALTITUDE_ERR and hdg_error < self.ACCEPTABLE_HEADING_ERR and not \
                    cur_eval["time to first contact s"]:
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
            trj_alt_error.append(alt_error if sim.get_property(prp.altitude_sl_ft) > des_alt else -alt_error)
            trj_hdg_error.append(hdg_error if sim.get_property(prp.heading_deg) < des_hdg else -hdg_error)
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
        exact_conditions = exact_conditions | {"altitude change desired": alt_change, 
        "heading change desired": hdg_change
        }

        cur_eval["avg airspeed"] /= num_steps
        cur_eval["avg load factor"] /= num_steps
        cur_eval["avg alt steady state error"] /= num_steps
        cur_eval["avg hdg steady state error"] /= num_steps

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
        self.trjs_alt_error.append(trj_alt_error)
        self.trjs_hdg_error.append(trj_hdg_error)
        self.trjs_kias.append(trj_kias)
        self.trjs_ground_speed.append(trj_ground_speed)
        self.trjs_time.append(trj_time)
        self.initial_conditions.append(exact_conditions)
        self.evals.append(cur_eval)
        self.cases.append((alt_case, hdg_case, wind_case))
        for var in trj_custom_tracking_vars:
            self.trjs_custom_tracking_vars[var].append(trj_custom_tracking_vars[var])

        return exact_conditions, cur_eval
    
    def batch_test(self, alt_cases: list[AltCase], hdg_cases: list[AltCase], wind_cases: list[WindCase], control_subsystem
    ,num_trials: int, aircraft:Aircraft=c172x):
        """Runs a complets batch of tests and saves all data to the instance varibales of the class. Does all
        combinations of the cases given. Prints out initial conditions and evaluations for each case. 
        
        :param alt_cases: List of altitude cases 
        :type alt_cases: list[AltCase]
        :param hdg_cases: List of heading cases
        :type hdg_cases: list[AltCase]
        :param wind_cases: list of wind cases
        :type wind_cases: list[WindCase]
        :param control_subsystem: An object that has a method action(SimulationInterface, des_alt, des_hdg)
        :type control_subsystem: object
        :param num_trials: The number of trials to run for each case
        :type num_trials: int
        :param aircraft: Aircraft type of type Aircraft, defaults to c172x
        :type aircraft: Aircraft, optional
        """

        for alt_case in alt_cases:
            for hdg_case in hdg_cases:
                for wind_case in wind_cases:
                    print(f"=== CASE ALTITUDE: {alt_case} HEADING: {hdg_case} WIND: {wind_case} ===")
                    for i in range(num_trials):
                        print(f"--> EXAMPLE {i}")
                        ics, evals = self.run_single_eval(alt_case, hdg_case, deepcopy(control_subsystem), aircraft=aircraft, wind_case=wind_case)
                        print(f"Average steady state error:{evals["avg alt steady state error"]}")
                        print(f"Time to first conatact: {evals["time to first contact s"]}") 
    
    def sort_evals(self) -> tuple[list[int], list[tuple]]:
        """Returns a list of the index of individual evalutaions sorted by from worst to best. It also changes the values of self.sorted_evals
        to the sorted_evals.
        The evaluation criteria is:
        1. Min airspeed below stall speed or max airspeed above Vne
        2. avg steady state alt error rounded to the nearest 50
        3. max alt overshoot to the nearest 50
        4. time to first contact greater than the expected time (true / false)
        5. max load factor to the nearest 0.5

        :return: indices, criteria. The inidices are The list of indexes of evaluations in self.evals. The criteria is a list
        of tuples describing how this function evaluated each eval as stated above. 
        :rtype: list[int]
        """
        def round_to_nearest(value, base):
            return round(value / base) * base

        def eval_sort_key(eval):
            # Criterion 1: Airspeed envelope violation
            airspeed_violation = (
                eval["min airspeed"] < self.aircraft.Vs1 or
                eval["max airspeed"] > self.aircraft.Vne
            )

            # Criterion 2: Avg alt steady-state error (rounded to nearest 50)
            avg_alt_error_rounded = round_to_nearest(eval["avg alt steady state error"], 50)

            # Criterion 3: Max alt overshoot (rounded to nearest 50)
            max_alt_overshoot_rounded = round_to_nearest(eval["max alt overshoot"], 50)

            # Criterion 4: Late contact time (True if time > expected)
            buffer_time_s = 5
            late_contact = eval["time to first contact s"] > (eval["max man time mins"] / 60 + buffer_time_s)

            # Criterion 5: Max load factor (rounded to nearest 0.5)
            max_load_factor_rounded = round_to_nearest(eval["max load factor"], 0.5)

            return (
                int(airspeed_violation),            # 0 = no violation, 1 = violation
                avg_alt_error_rounded,
                max_alt_overshoot_rounded,
                int(late_contact),                  # 0 = on-time or earlier, 1 = late
                max_load_factor_rounded
            )

        indexed_evals = list(enumerate(self.evals))
        indexed_evals.sort(key=lambda pair: eval_sort_key(pair[1]))
        indexed_evals.reverse() # Reverse from best to worst to worst to best

        self.sorted_indices = [idx for idx, _ in indexed_evals]
        criteria = [eval_sort_key(pair[1]) for pair in indexed_evals]
        return self.sorted_indices, criteria
    
    def create_batch_eval(self, indices: list[int], plot=True):
        """Creates an evaluation for the batch of indicies given. The batch evaluation will invlude the following, with the case and number:
        batch_evals = {
            "max alt overshoot": 0, 
            "max alt avg steady state error": 0,
            "avg alt steady state error": 0,
            "avg max alt overshoot": 0,
            "avg hdg steady state error": 0,
            "max hdg overshoot": 0,
            "avg time fufillment": 0, # This is the average of fraction of the max man time the algorithm takes to first contant 
            "mean abs alt steady state error": 0
        }

        If plot is true, then it will create a plot of max alt overshoot and avg alt ss error for all cases.
        """
        batch_evals = {
            "max alt overshoot": 0, 
            "max alt avg steady state error": 0,
            "avg alt steady state error": 0,
            "avg max alt overshoot": 0,
            "avg hdg steady state error": 0,
            "max hdg overshoot": 0,
            "avg time fufillment": 0, # This is the average of fraction of the max man time the algorithm takes to first contant 
            "mean abs alt steady state error": 0
        }
        for i in indices:
            batch_evals["max alt overshoot"] = max(batch_evals["max alt overshoot"], self.evals[i]["max alt overshoot"])
            batch_evals["max alt avg steady state error"] = max(batch_evals["max alt avg steady state error"], self.evals[i]["avg alt steady state error"])
            batch_evals["avg alt steady state error"] += self.evals[i]["avg alt steady state error"]
            batch_evals["avg max alt overshoot"] += self.evals[i]["max alt overshoot"]
            batch_evals["avg hdg steady state error"] += self.evals[i]["avg hdg steady state error"]
            batch_evals["max hdg overshoot"] = max(batch_evals["max hdg overshoot"], self.evals[i]["max hdg overshoot"])
            if self.evals[i]["max man time mins"] != 0:
                time_first_c = self.evals[i]["time to first contact s"]
                if time_first_c == 0:
                    time_first_c = self.evals[i]["max man time mins"] * 2 * 60
                batch_evals["avg time fufillment"] +=  time_first_c / self.evals[i]["max man time mins"] / 60
        num_indices = len(indices)
        batch_evals["avg alt steady state error"] /= num_indices
        batch_evals["avg max alt overshoot"] /= num_indices
        batch_evals["avg hdg steady state error"] /= num_indices
        batch_evals["avg time fufillment"] /= num_indices
        for i in indices:
            batch_evals["mean abs alt steady state error"] += abs(self.evals[i]["avg alt steady state error"] - batch_evals["avg alt steady state error"])
        batch_evals["mean abs alt steady state error"] /= num_indices
        return batch_evals

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
        axs[0].plot(time, self.trjs_elevator_ap_cmd[idx], label="Elevator AP Cmd")
        axs[0].plot(time, self.trjs_elevator_pos[idx], label="Elevator Pos Rad")
        axs[0].plot(time, self.trjs_elevator_trim[idx], label="Elevator Trim")
        # axs[0].plot(time, self.trjs_rudder_fcs_cmd[idx], label="Rudder FCS Cmd")
        # axs[0].plot(time, self.trjs_rudder_fcs_pos[idx], label="Rudder FCS Pos")
        axs[0].set_title("All Control Commands and Deflections")
        axs[0].set_ylabel("Normalized Value")
        axs[0].legend()
        axs[0].grid(True, which='both')

        # Second chart: altitude error
        axs[1].plot(time, self.trjs_alt_error[idx], label="Altitude Error [ft]")
        axs[1].set_title("Altitude Error")
        axs[1].set_ylabel("Error")
        axs[1].legend()
        axs[1].axvline(x=self.evals[index]["max man time mins"]*60)
        axs[1].grid(True, which='both')

        # Third chart: heading error
        axs[2].plot(time, self.trjs_hdg_error[idx], label="Heading Error [deg]")
        axs[2].set_title("Heading Error")
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