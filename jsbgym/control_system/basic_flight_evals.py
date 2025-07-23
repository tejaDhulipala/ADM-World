from math import pi
from typing import Dict

from ..aircraft import *
from ..control_system.control_system_default import AltitudeHoldSubsystem, HeadingHoldSubsystem
from ..simulation_interface import SimulationInterface
from .. import properties as prp
import random
from enum import Enum


class AltCase(Enum):
    ALT_HOLD = "hold altitude"
    CLB_L_200 = "climb less than 200 ft"
    CLB_200_500 = "climb between 200 and 500 ft"
    CLB_G_500 = "climb above 500 ft"
    DSC_L_200 = "descend less than 200 ft"
    DSC_200_500 = "descend between 200 and 500 ft"
    DSC_G_500 = "descend above 500 ft"

class HdgCase(Enum):
    HDG_HOLD = "hold heading"
    HDG_L_45 = "change heading by less than 45 degrees"
    HDG_45_90 = "change heading between 45 and 90 degrees"
    HDG_90_180 = "change heading between 90 and 180 degrees"

class WindCase(Enum):
    CLM = "No Winds"
    WND_L_5 = "Max crosswind component of less than 5 knots. Steady wind."
    WND_5_10 = "Max crosswind component between 5 and 10 knots. Steady wind."
    WND_10_15 = "Max crosswind component between 10 and 15 knots. Steady wind."
    WND_G_15 = "Max crosswind component greater than 15 knots. Steady."

class BasicFlightControlEval:
    """
    The purpose of this class is to test and produce evaluation metrics of the ability of the airplane to do any combination of the four basic flight
    maneuvers: climbs, descents, turns, and straight & level in the normal (not slow flight) control regime. 
    """
    ACCEPTABLE_HEADING_ERR = 7.5
    ACCEPTABLE_ALTITUDE_ERR = 50
    def __init__(self) -> None:
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
            "min airspeed": initial_airspeed_fps / Aircraft.KTS_TO_FT_PER_S,
            "max airspeed": -1,
            "avg airspeed": 0,
            "avg alt steady state error": 0,
            "avg hdg steady state error": 0,
            "time to first contact s": -1
        }
        :rtype: Dict
        """
        # --> Ensure correct formatting
        assert isinstance(alt_case, AltCase)
        assert isinstance(hdg_case, HdgCase)
        assert isinstance(wind_case, WindCase)
        try:
            sim = SimulationInterface()
            sim.initialize()
            act = control_subsystem.action(sim, 5000, 90)
            assert type(act) == dict
        except:
            raise AssertionError("The altitude or heading algorithm is not is the correct format")
        
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
                alt_case = random.uniform(700, 3000)
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
            prp.initial_altitude_ft: initial_alt,
            prp.initial_heading_deg: initial_hdg,
            prp.initial_u_fps: initial_airspeed_fps
        }
        interaction_freq = 5

        # Calculate the steps of the simulation so it doesn't render more than it needs to & determine steps to consider steady state
        max_maneuver_time_mins = max(alt_change / 500, hdg_change / 180) * 1.5
        max_man_steps = max_maneuver_time_mins * 60 * interaction_freq
        max_steps = max_man_steps + 90 * interaction_freq # 90 seconds of steady state flight 
        print(f"Running simulatin for {max_steps} steps")

        # Set-up eval metrics dictionary
        cur_eval = {
            "max alt overshoot": 0,
            "max hdg overshoot": 0,
            "max load factor": 0,
            "avg load factor": 0,
            "min airspeed": initial_airspeed_fps / Aircraft.KTS_TO_FT_PER_S,
            "max airspeed": -1,
            "avg airspeed": 0,
            "avg alt steady state error": 0,
            "avg hdg steady state error": 0,
            "time to first contact s": -1
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
        
        # --> Run sim
        sim = SimulationInterface(initial_conditions=initial_conditions, aircraft=aircraft, render_mode=render_mode,
        control_agent_interaction_freq=interaction_freq) # render mode can be None, flightgear, human, graph, human_fg, or graph_fg
        sim.initialize()

        num_steps = 0
        while num_steps < max_steps:
            # --> Execute actions
            actions = {}
            for prop, val in control_subsystem.action(sim, des_alt, des_hdg).items():
                actions[prop] = val
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
            cur_eval["min airspeed"] = min(cur_eval["min airspeed"], sim.get_property(prp.ias_kts))
            # max airspeed
            cur_eval["max airspeed"] = max(cur_eval["max airspeed"], sim.get_property(prp.ias_kts))
            # avg airspeed, but sum for now
            cur_eval["avg airspeed"] += sim.get_property(prp.ias_kts)
            # avg alt steady state error
            if num_steps > max_man_steps:
                cur_eval["avg alt steady state error"] += alt_error
            # avg hdg steady state error
            if num_steps > max_man_steps:
                cur_eval["avg hdg steady state error"] += alt_error
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
            trj_elevator_pos.append(sim.get_property(prp.elevator))
            # Errors
            trj_alt_error.append(alt_error if sim.get_property(prp.altitude_sl_ft) > des_alt else -alt_error)
            trj_hdg_error.append(hdg_error if sim.get_property(prp.heading_deg) < des_hdg else -hdg_error)
            # Speeds
            trj_kias.append(sim.get_property(prp.ias_kts))
            trj_ground_speed.append(sim.get_property(prp.groundspeed_kts))
            # Time
            trj_time.append(sim.get_property(prp.sim_time_s))

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

        return exact_conditions, cur_eval

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
        fig, axs = plt.subplots(4, 1, figsize=(14, 16), sharex=True)

        # First chart: All control commands and deflections
        axs[0].plot(time, self.trjs_aileron_pos_left[idx], label="Aileron Pos Left")
        axs[0].plot(time, self.trjs_aileron_pos_right[idx], label="Aileron Pos Right")
        # axs[0].plot(time, self.trjs_aileron_fcs_cmd[idx], label="Aileron FCS Cmd")
        # axs[0].plot(time, self.trjs_aileron_ap_cmd[idx], label="Aileron AP Cmd")
        # axs[0].plot(time, self.trjs_elevator_fcs_cmd[idx], label="Elevator FCS Cmd")
        axs[0].plot(time, self.trjs_elevator_ap_cmd[idx], label="Elevator AP Cmd")
        axs[0].plot(time, self.trjs_elevator_pos[idx], label="Elevator Pos Norm")
        # axs[0].plot(time, self.trjs_elevator_trim[idx], label="Elevator Trim")
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
        axs[1].grid(True, which='both')

        # Third chart: heading error
        axs[2].plot(time, self.trjs_hdg_error[idx], label="Heading Error [deg]")
        axs[2].set_title("Heading Error")
        axs[2].set_ylabel("Error")
        axs[2].legend()
        axs[2].grid(True, which='both')
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

        plt.tight_layout()
        plt.minorticks_on()
        plt.show()

class BasicFlightControlSubsystem:
    def __init__(self) -> None:
        self.heading_subsystem = HeadingHoldSubsystem()
        self.altitude_subsystem = AltitudeHoldSubsystem()

    def action(self, sim: SimulationInterface, des_alt, des_hdg):
        assert des_hdg <= 360
        actions = {}
        for prop, val in self.heading_subsystem.action(des_hdg).items():
            actions[prop] = val
        for prop, val in self.altitude_subsystem.action(des_alt).items():
            actions[prop] = val
        
        # Set the throttle and mixture 
        alt_change = des_alt - sim.get_property(prp.altitude_sl_ft)
        if  alt_change >= 150:
            actions[prp.throttle_cmd] = 1.0
        elif alt_change >= -150:
            actions[prp.throttle_cmd] = 2300 / 2700
        else:
            actions[prp.throttle_cmd] = 1800 / 2700
        actions[prp.mixture_cmd] = 0.8

        return actions

if __name__ == "__main__":
    basic_flight_control_evaluator = BasicFlightControlEval()
    conditions, flight_evals = basic_flight_control_evaluator.run_single_eval(AltCase.DSC_G_500, HdgCase.HDG_L_45, BasicFlightControlSubsystem())
    print(conditions)
    print(flight_evals)
    basic_flight_control_evaluator.plot_eval()