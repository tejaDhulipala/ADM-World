from copy import deepcopy
from math import pi
from typing import Tuple
from ...simulation_interface import SimulationInterface
from ... import properties as prp
from .pid_controller import PIDController
from numpy import clip, sign
import matplotlib.pyplot as plt

class RollSubsystem:
    def __init__(self) -> None:
        pass
    
    def action(self, sim: SimulationInterface, des_roll):
        return {
            prp.ap_hold_straight_and_level: 1
        }

class FlightPathPIDSubsystem:
    def __init__(self) -> None:
        self.flight_path_hold_pid = PIDController(0.071, 0.0055, 0.0)
        self.throttle_pid = PIDController(0.01, 0.001, 0)
        self.min_cmd = -0.5
        self.max_cmd = 0.5
        self.num_steps = 0
        self.epsilon_trim_flight_path = 0.055 # How close the aircraft should get to the desired pitch before trimming in degrees
        self.min_delta_trim = 1 # the minimum change in desired flight path angle to retrim
        self.target_airspeed = 80
        self.trim_throttle = 1.0

        self._last_time = None
        self._last_trim_fpa = -1000 # fpa means flight path angle 
    
    def action(self, sim: SimulationInterface, des_flight_path):
        actions = {
            prp.elevator_cmd: 0
        }
        
        if self._last_time == None:
            self._last_time = sim.get_property(prp.sim_time_s)
        
        dt = sim.get_property(prp.sim_time_s) - self._last_time
        # The idea is to do the PID on the projected pitch so it doesn't overshoot 
        # Positive = elevator down = nose down
        flight_path = sim.get_property(prp.flight_path)
        elevator_cmd = -self.flight_path_hold_pid.compute(flight_path, des_flight_path, dt)
        elevator_cmd = clip(elevator_cmd, self.min_cmd, self.max_cmd) if self._last_trim_fpa < -999 else 0

        # Autothrottle
        throttle = sim.get_property(prp.throttle_cmd) + \
            clip(self.throttle_pid.compute(sim.get_property(prp.cas_kts), self.target_airspeed, dt) * dt, -1, 1)

        # Change variables
        self._last_time = sim.get_property(prp.sim_time_s)
        actions[prp.elevator_cmd] = elevator_cmd
        print(throttle)
        actions[prp.throttle_cmd] = throttle

        self.num_steps += 1

        return actions

class RPPIDControlSubsystem:
    def __init__(self, roll_subsystem=RollSubsystem(), pitch_subsystem=FlightPathPIDSubsystem()) -> None:
        self.roll_subsystem = roll_subsystem
        self.pitch_subsystem = pitch_subsystem
        self.num_steps = 0
        self.trim_throttle = None
        self.trim_pitch = None
        self.trim_aoa = None
    
    def action(self, sim: SimulationInterface, des_pitch, des_roll):
        assert des_roll <= 360
        actions = {}

        # Do stuff based on the number of steps
        if self.num_steps == 0:
            actions["simulation/do_simple_trim"] = 0
            print("trim done")
        if self.num_steps == 1:
            self.trim_throttle = sim.get_property(prp.throttle_cmd)
            self.trim_pitch = sim.get_property(prp.pitch_rad)
            self.trim_aoa = sim.get_property(prp.angle_of_attack)
            self.pitch_subsystem.trim_throttle = self.trim_throttle
            print(self.trim_pitch)

        # Set flight commands
        for prop, val in self.roll_subsystem.action(sim, des_roll).items():
            actions[prop] = val
        for prop, val in self.pitch_subsystem.action(sim, des_pitch).items():
            actions[prop] = val
        
        # Increment num steps
        self.num_steps += 1

        return actions


def run_cmds(sim: SimulationInterface, elev_cmd: float, time: float = 90, graph:bool=False) -> Tuple[float, float, bool, bool, bool]:
    """This functions takes in a simulation and will deepcopy it immediately. Runs the commands and produces an output.

    :param sim: SimulationInterface
    :type sim: SimulationInterface
    :param elev_cmd: the elevator cmd. positive is down, negative is up
    :type elev_cmd: float
    :param time: The time in seconds the simulation should run for. Default is 90 seconds
    :type time: float
    :return: Returns a tuple of avg flight path in deg, avg calibrated airspeed, stall(t/f), overspeed(t/f), crash(t/f)
    :rtype: Tuple[float, float, bool, bool, bool]
    """
    sim_new = deepcopy(sim)
    control_subsystem = RPPIDControlSubsystem(pitch_subsystem=FlightPathFixedSubsystem(elev_cmd, sim_new.get_property(prp.throttle)))
    num_steps = 0
    max_steps = time * sim_new.control_agent_interaction_freq
    flight_path_sum = 0
    cas_sum = 0
    stall = False
    overspeed = False
    crash = False
    crash_altitude = 5
    # Initialize data storage
    time_data = []
    altitude_data = []
    airspeed_data = []
    pitch_data = []
    flight_path_data = []

    while num_steps < max_steps:
        actions = control_subsystem.action(sim_new, 90, 90)
        sim_new.step(actions)
        
        flight_path_sum += sim_new.get_property(prp.flight_path)
        cas_sum += sim_new.get_property(prp.cas_kts)
        if sim_new.get_property(prp.cas_kts) < sim_new.aircraft.Vs0:
            stall = True
        if sim_new.get_property(prp.cas_kts) > sim_new.aircraft.Vne:
            overspeed = True
        if sim_new.get_property(prp.altitude_agl_ft) < crash_altitude:
            crash = True
        
        # Store data
        sim_time = sim_new.get_property(prp.sim_time_s)
        time_data.append(sim_time)
        altitude_data.append(sim_new.get_property(prp.altitude_sl_ft))
        airspeed_data.append(sim_new.get_property(prp.cas_kts))
        pitch_data.append(sim_new.get_property(prp.pitch_rad) * 180/pi)
        flight_path_data.append(sim_new.get_property(prp.flight_path))

        num_steps += 1
    
    print(f"Elevator command: {elev_cmd}, Avg flight path: {flight_path_sum / num_steps}")

    if graph:
        # Create subplots
        _, axs = plt.subplots(4, 1, figsize=(12, 16), sharex=True)
        
        # Altitude plot
        axs[0].plot(time_data, altitude_data, 'b-', label='Altitude')
        axs[0].set_ylabel('Altitude (ft)')
        axs[0].grid(True)
        axs[0].legend()

        # Airspeed plot
        axs[1].plot(time_data, airspeed_data, 'g-', label='Airspeed')
        axs[1].set_ylabel('CAS (KTS)')
        axs[1].grid(True)
        axs[1].legend()

        # Pitch angle plot
        axs[2].plot(time_data, pitch_data, 'r-', label='Pitch')
        axs[2].set_ylabel('Angle (°)')
        axs[2].grid(True)
        axs[2].legend()

        # Flight path angle plot
        axs[3].plot(time_data, flight_path_data, 'm-', label='Flight Path')
        axs[3].set_xlabel('Time (s)')
        axs[3].set_ylabel('Angle (°)')
        axs[3].grid(True)
        axs[3].legend()

        plt.tight_layout()
        plt.show()
    
    return (flight_path_sum / num_steps, cas_sum / num_steps, stall, overspeed, crash)

def trim_in_flight(sim: SimulationInterface, des_flight_path, max_guesses=10, des_err=0.05):
    """The simulation interface WILL be deepcopied in this function for safety and encapsulation. The simulation must be 
    trimmed and initialized. It returns the elevator trim required to maintain the current flight path while leveling wings. 
    """
    upper_bound_elv_cmd = 1
    lower_bound_elv_cmd = -1
    set_cmd_flight_path = 1000 # arbitrary
    set_elev_cmd = 1000
    guesses = 0

    while abs(set_cmd_flight_path) > des_err and guesses < max_guesses:
        set_elev_cmd = (upper_bound_elv_cmd + lower_bound_elv_cmd) / 2
        set_cmd_flight_path, _, _, _, _ = run_cmds(sim, set_elev_cmd)
        if set_cmd_flight_path > des_flight_path:
            lower_bound_elv_cmd = set_elev_cmd
        else:
            upper_bound_elv_cmd = set_elev_cmd
        
        guesses += 1
    
    return -0.110 + sim.get_property(prp.pitch_trim)

class FlightPathFixedSubsystem:
    def __init__(self, elevator_cmd, throttle_cmd=None) -> None:
        self.num_steps = 0    
        self.elevator_cmd = elevator_cmd
        self.throttle_cmd = throttle_cmd
    def action(self, sim: SimulationInterface, des_pitch):
        actions = {
            prp.elevator_cmd: self.elevator_cmd
        }

        if self.throttle_cmd != None:
            actions[prp.throttle_cmd] = self.throttle_cmd      

        self.num_steps += 1

        return actions