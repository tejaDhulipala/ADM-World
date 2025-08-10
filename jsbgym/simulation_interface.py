from math import pi

import jsbsim
from . import properties as prp
from .simulation import Simulation
from .visualiser import FigureVisualiser, FlightGearVisualiser, GraphVisualiser
from .aircraft import Aircraft, c172x
from .properties import BoundedProperty, Property
from typing import Optional, Dict, Union
import warnings

class SimulationInterface:
    """
    An interface allowing for rendering of the simulation and inputting actions to the simulation.
    It is essentially the Simulation plus rendering   
    """

    JSBSIM_DT_HZ: int = 60  # JSBSim integration frequency
    metadata = {
        "render_modes": ["human", "flightgear", "human_fg", "graph", "graph_fg"],
        "render_fps": 60,
    }
    base_observation_variables = [
        prp.lat_geod_deg,         # latitude [deg]
        prp.lng_geoc_deg,         # longitude [deg]
        prp.altitude_sl_ft,       # altitude MSL [ft]
        prp.altitude_agl_ft,      # altitude AGL [ft]
        prp.heading_deg,

        # --- Plane Orientation ---
        prp.pitch_rad,            # pitch [rad]
        prp.roll_rad,             # roll [rad]
        prp.sideslip_deg,         # sideslip [deg]

        # --- Velocities ---
        prp.cas_kts,              # indicated airspeed [kts]
        prp.groundspeed_fps,      # groundspeed [ft/s]
        prp.vertical_speed_fps,    # vertical speed [ft/s]

        # --- Control state ---
        prp.aileron_left,
        prp.aileron_right,
        prp.elevator,
        prp.rudder,
        # --- Engine state ---
        prp.engine_running,
        prp.throttle,
        prp.mixture_cmd,
        prp.oil_pressure_psi,
        prp.oil_temperature_degF,
        # --- Autopilot ---
        prp.ap_hold_straight_and_level,
        prp.ap_hold_altitude,
        prp.ap_hold_heading,
        prp.ap_heading_setpoint,
        prp.ap_altitude_setpoint,
        # Time
        prp.sim_dt
    ]
    default_state_variables = {
        prp.initial_u_fps: 0,
        prp.initial_v_fps: 0,
        prp.initial_w_fps: 0,
        prp.initial_p_radps: 0,
        prp.initial_q_radps: 0,
        prp.initial_r_radps: 0,
        prp.initial_roc_fpm: 0,
        prp.initial_heading_deg: 0,
    }
    default_initial_conditions = {
            prp.initial_altitude_msl_ft: 5000,
            prp.initial_terrain_altitude_ft: 0.00000001,
            prp.initial_longitude_geoc_deg: -2.3273,
            prp.initial_latitude_geod_deg: 51.3781,  # corresponds to UoBath
    }

    default_initial_conditions = default_initial_conditions | default_state_variables

    def __init__(
        self,
        aircraft: Aircraft = c172x,
        initial_conditions: Dict = {},
        additional_obervation_properties: list = [],
        control_agent_interaction_freq: int = 20,
        render_mode: Optional[str] = None,
        debug_props: bool = False
    ):
        """A Simulation Interface is essentially just a simulation with rendering. Init just
        sets variables. initialize() actually starts the simulation.

        :param aircraft: Aircraft type, defaults to c172x
        :type aircraft: Aircraft, optional
        :param initial_conditions: dictionary of initial conditions. The example is shown in this class. If the 
        inputed inital conditions are incomplete, it uses the default values, defaults to default_initial_conditions
        :type initial_conditions: _type_, optional
        :param control_agent_interaction_freq: How many times a second the control agent can interact, defaults to 5
        :type control_agent_interaction_freq: int, optional
        :param render_mode: human_fg, flightgear, graph_fg, human, graph, other, defaults to None
        :type render_mode: Optional[str], optional
        :param debug_props: Tells you what properties you have and have not changed on init
        :type render_mode: bool, optional
        :raises ValueError: _description_
        """
        if control_agent_interaction_freq > self.JSBSIM_DT_HZ:
            raise ValueError(
                "agent interaction frequency must be less than "
                "or equal to JSBSim integration frequency of "
                f"{self.JSBSIM_DT_HZ} Hz."
            )
        self.sim: Simulation = None
        self.sim_steps_per_agent_step: int = self.JSBSIM_DT_HZ // control_agent_interaction_freq
        self.control_agent_interaction_freq = control_agent_interaction_freq
        self.aircraft = aircraft
        # set visualisation objects
        self.figure_visualiser: FigureVisualiser = None
        self.flightgear_visualiser: FlightGearVisualiser = None
        self.graph_visualiser: GraphVisualiser = None
        self.step_delay = None
        self.render_mode = render_mode
        self.init_conditions = SimulationInterface.default_initial_conditions.copy()
        # Print initial conditions that the user did not modify 
        self.init_conditions[prp.initial_u_fps] = self.aircraft.get_cruise_speed_fps()
        user_keys = set(initial_conditions.keys())
        if debug_props:
            init_condition_keys = set(SimulationInterface.default_initial_conditions.keys())
            print(f"Properties that you did not add (set to default): {init_condition_keys - user_keys}")
            print(f"Properties changed: {init_condition_keys.intersection(user_keys)}")
            print(f"Additional properties you added that defaults did not already have {user_keys - init_condition_keys}")
        for prop, val in initial_conditions.items():
            self.init_conditions[prop] = val
        # Add additional observation properties
        self.observation_variables = SimulationInterface.base_observation_variables + additional_obervation_properties
        self.observation_variables = tuple(set(self.observation_variables))
        # Simulation environment
        self.time_series = []
        self.aileron_cmd = []
        self.elevator_vars = []
        self.rudder_cmd = []
        self.observations = []

    def step(self, actions: Dict[Union[Property, BoundedProperty, str], Union[int, float]]):
        if (
            self.render_mode == "human"
            or self.render_mode == "graph"
            or self.render_mode == "human_fg"
            or self.render_mode == "graph_fg"
            or self.render_mode == "flightgear"
        ):
            self.render()

        for prop, command in actions.items():
            try:
                self.sim[prop] = command
            except jsbsim.TrimFailureError:
                print("Trim failed")
        
        # Data collection for graphing
        self.time_series.append(self.sim[prp.sim_time_s])
        self.aileron_cmd.append(self.sim[prp.aileron_left])
        elevator_props = [self.sim["fcs/elevator-cmd-norm"], self.sim["fcs/elevator-pos-rad"], 
        self.sim["ap/elevator_cmd"], self.sim["fcs/pitch-trim-cmd-norm"]]
        self.elevator_vars.append(elevator_props)
        self.rudder_cmd.append(self.sim[prp.rudder_cmd])
        ob = {prop: self.sim[prop] for prop in self.observation_variables}
        self.observations.append(ob)

        # run simulation
        for _ in range(self.sim_steps_per_agent_step):
            self.sim.run()
        
        return ob

    def initialize(self) -> Dict[Union[BoundedProperty, Property], Union[float, int]]:
        if self.sim:
            self.sim.reinitialise(self.init_conditions)
        else:
            self.sim = self._init_new_sim(
                self.JSBSIM_DT_HZ, self.aircraft, self.init_conditions, self.render_mode in ["flightgear", "human_fg", "graph_fg"]
            )

        if self.flightgear_visualiser:
            self.flightgear_visualiser.configure_simulation_output(self.sim)
        if self.render_mode == "human":
            self.render()
        if self.render_mode == "graph":
            try:
                self.graph_visualiser.reset()
            except AttributeError:
                pass
        
        # Starts the engines
        self.sim.start_engines()
        self.sim.raise_landing_gear()

        return {prop: self.sim[prop] for prop in self.observation_variables} 

    def _init_new_sim(self, dt, aircraft, initial_conditions, using_flightgear):
        return Simulation(
            sim_frequency_hz=dt, aircraft=aircraft, init_conditions=initial_conditions, allow_flightgear_output=using_flightgear
        )

    def render(self, flightgear_blocking=True):
        if self.render_mode is None:
            assert self.spec is not None
            warnings.warn(
                "You are calling render method without specifying any render mode"
            )
            return

        """Renders the environment.
        The set of supported modes varies per environment. (And some
        environments do not support rendering at all.) By convention,
        if mode is:
        - human: render to the current display or terminal and
          return nothing. Usually for human consumption.
        - rgb_array: Return an numpy.ndarray with shape (x, y, 3),
          representing RGB values for an x-by-y pixel image, suitable
          for turning into a video.
        - ansi: Return a string (str) or StringIO.StringIO containing a
          terminal-style text representation. The text can include newlines
          and ANSI escape sequences (e.g. for colors).
        Note:
            Make sure that your class's metadata 'render.modes' key includes
              the list of supported modes. It's recommended to call super()
              in implementations to use the functionality of this method.

        :param mode: str, the mode to render with
        :param flightgear_blocking: waits for FlightGear to load before
            returning if True, else returns immediately
        """
        six_pack = (prp.cas_kts, prp.altitude_sl_ft, prp.heading_deg, prp.pitch_rad, prp.pitch_rad, prp.roll_rad)

        if self.render_mode == "human":
            if not self.figure_visualiser:
                self.figure_visualiser = FigureVisualiser(
                    self.sim, six_pack
                )
            self.figure_visualiser.plot(self.sim)
        elif self.render_mode == "flightgear":
            if not self.flightgear_visualiser:
                self.flightgear_visualiser = FlightGearVisualiser(
                    self.sim, self.observation_variables, flightgear_blocking
                )
        elif self.render_mode == "human_fg":
            if not self.flightgear_visualiser:
                self.flightgear_visualiser = FlightGearVisualiser(
                    self.sim, six_pack, flightgear_blocking
                )
            self.flightgear_visualiser.plot(self.sim)
        elif self.render_mode == "graph":
            if not self.graph_visualiser:
                self.graph_visualiser = GraphVisualiser(
                    self.sim, self.observation_variables
                )
            self.graph_visualiser.plot(self.sim)
        elif self.render_mode == "graph_fg":
            if not self.flightgear_visualiser:
                self.flightgear_visualiser = FlightGearVisualiser(
                    self.sim, self.observation_variables, flightgear_blocking
                )
            if not self.graph_visualiser:
                self.graph_visualiser = GraphVisualiser(
                    self.sim, self.observation_variables
                )
            self.graph_visualiser.plot(self.sim)
    
    def get_property(self, prop: Union[BoundedProperty, Property, str]):
        return self.sim[prop]

    def close(self):
        """Cleans up this environment's objects

        Environments automatically close() when garbage collected or when the
        program exits.
        """
        if self.sim:
            self.sim.close()
        if self.figure_visualiser:
            self.figure_visualiser.close()
        if self.flightgear_visualiser:
            self.flightgear_visualiser.close()
        if self.graph_visualiser:
            self.graph_visualiser.close()
    
    def __deepcopy__(self, memo):
        ics = {
            prp.initial_altitude_msl_ft: self.sim[prp.altitude_sl_ft],
            prp.initial_terrain_altitude_ft: self.sim[prp.initial_terrain_altitude_ft],
            prp.initial_longitude_geoc_deg: self.sim[prp.lng_geoc_deg],
            prp.initial_latitude_geod_deg: self.sim[prp.lat_geod_deg],
            prp.initial_u_fps: self.sim[prp.u_fps],
            prp.initial_v_fps: self.sim[prp.v_fps],
            prp.initial_w_fps: self.sim[prp.w_fps],
            prp.initial_p_radps: self.sim[prp.p_radps],
            prp.initial_q_radps: self.sim[prp.q_radps],
            prp.initial_r_radps: self.sim[prp.initial_r_radps],
            prp.initial_heading_deg: self.sim[prp.heading_deg],
            prp.initial_wind_heading: self.sim[prp.initial_wind_heading],
            prp.initial_wind_speed: self.sim[prp.initial_wind_speed],
            prp.initial_pitch_deg: self.sim[prp.pitch_rad] / pi * 180,
            prp.initial_roll_deg: self.sim[prp.roll_rad] / pi * 180,
            prp.initial_flight_path: self.sim[prp.flight_path],
            "ic/vc-kts": self.sim[prp.cas_kts],
            "ic/alpha-deg": self.sim[prp.angle_of_attack],
            "ic/beta-deg": self.sim[prp.sideslip_deg],
        }

        # Create new instance with current initial conditions
        new_sim = SimulationInterface(
            aircraft=self.aircraft, 
            initial_conditions=ics,
            control_agent_interaction_freq=self.control_agent_interaction_freq,
            render_mode="", 
        )
        
        # Initialize the new simulation
        new_sim.initialize()
        
        return new_sim