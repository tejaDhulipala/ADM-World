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
    CLB_L_100 = "climb less than 100 ft"
    CLB_G_100 = "climb greater than 100 ft"
    DSC_L_100 = "descend less than 100 ft"
    DSC_G_100 = "descend greater than 100 ft"

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
        pass

    def run_single_eval(self, alt_case: AltCase, hdg_case: HdgCase, control_subsystem,
     render_mode=None, aircraft:Aircraft=c172x, wind_case:WindCase=WindCase.CLM) -> Dict:
        """_summary_

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
        :return: A dictionary of evaluations methods, which include:
        {
            "max overshoot": 0,
            "max load factor": 0,
            "avg load factor": 0,
            "min airspeed": 0,
            "max airspeed": 0,
            "avg airspeed": 0,
            "avg steady state error": 0,
            "time to first contact s": 0
        }
        :rtype: Dict
        """
        # Ensure correct formatting
        assert isinstance(alt_case, AltCase)
        assert isinstance(hdg_case, HdgCase)
        assert isinstance(wind_case, WindCase)
        try:
            act = control_subsystem.action(SimulationInterface(), 5000, 90)
            assert type(act) == dict
        except:
            raise AssertionError("The altitude or heading algorithm is not is the correct format")
        
        # Change variables based on the case
        alt_change = 0
        hdg_change = 0
        wind_speed = 0
        match alt_case:
            case AltCase.ALT_HOLD:
                alt_change = 0
            case AltCase.CLB_L_100:
                alt_change = random.uniform(20, 100)
            case AltCase.CLB_G_100:
                alt_change = random.uniform(100, 3000)
            case AltCase.DSC_L_100:
                alt_case = -random.uniform(20, 100)
            case AltCase.DSC_G_100:
                alt_change = -random.uniform(100, 3000)
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
            "max overshoot": 0,
            "max load factor": 0,
            "avg load factor": 0,
            "min airspeed": initial_airspeed_fps / Aircraft.KTS_TO_FT_PER_S,
            "max airspeed": -1,
            "avg airspeed": 0,
            "avg steady state error": 0,
            "time to first contact s": 0
        }

        # Run sim
        sim = SimulationInterface(initial_conditions=initial_conditions, aircraft=aircraft, render_mode=render_mode,
        control_agent_interaction_freq=interaction_freq) # render mode can be None, flightgear, human, graph, human_fg, or graph_fg
        sim.initialize()

        num_steps = 0
        while num_steps < max_steps:
            # Execute actions
            actions = {}
            for prop, val in control_subsystem.action(sim, des_alt, des_hdg).items():
                actions[prop] = val
            obs = sim.step(actions)
            
            # Edit evaluations
            # Edit max overshoot
            overshoot = abs(des_alt - sim.get_property(prp.altitude_sl_ft))
            if alt_change < 0 and sim.get_property(prp.altitude_sl_ft) < des_alt:
                cur_eval["max overshoot"] = max(cur_eval["max overshoot"], overshoot)
            elif alt_change > 0 and sim.get_property(prp.altitude_sl_ft) > des_alt:
                cur_eval["max overshoot"] = max(cur_eval["max overshoot"], overshoot)
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
            # Avg steady state error
            if num_steps > max_man_steps:
                cur_eval["avg steady state error"] += overshoot
            # Time to first contact
            if abs(sim.get_property(prp.altitude_sl_ft) - des_alt) < self.ACCEPTABLE_ALTITUDE_ERR and \
                abs(sim.get_property(prp.heading_deg) - des_hdg) < self.ACCEPTABLE_HEADING_ERR and not \
                    cur_eval["time to first contact s"]:
                    cur_eval["time to first contact s"] = num_steps / interaction_freq

            # increment steps
            num_steps += 1
        sim.close()

        # Return the correct evaluation 
        cur_eval["avg airspeed"] /= num_steps
        cur_eval["avg load factor"] /= num_steps
        cur_eval["avg steady state error"] /= num_steps

        return cur_eval

class BasicFlightControlSubsystem:
    def __init__(self) -> None:
        self.heading_subsystem = HeadingHoldSubsystem()
        self.altitude_subsystem = AltitudeHoldSubsystem()

    def action(self, sim: SimulationInterface, des_alt, des_hdg):
        assert des_hdg <= 360
        actions = {}
        for prop, val in self.heading_subsystem.action(des_hdg):
            actions[prop] = val
        for prop, val in self.altitude_subsystem.action(des_alt):
            actions[prop] = val
        
        # Set the throttle and mixture 
        alt_change = des_alt - sim.get_property(prp.altitude_sl_ft)
        if  alt_change >= 150:
            actions[prp.throttle_cmd] = 1.0
        elif alt_change >= -150:
            actions[prp.throttle_cmd] = 2400 / 2700
        else:
            actions[prp.throttle_cmd] = 1800 / 2700

if __name__ == "__main__":
    basic_flight_control_evaluator = BasicFlightControlEval()
    flight_evals = basic_flight_control_evaluator.run_single_eval(AltCase.ALT_HOLD, HdgCase.HDG_HOLD)
    print(flight_evals)