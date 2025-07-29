from .basic_flight_evals import *
from .basic_flight_pid import *

basic_flight_evaluator = BasicFlightControlEval()
sim = SimulationInterface()
sim.initialize()
basic_flight_evaluator.run_single_eval(AltCase.ALT_HOLD, HdgCase.HDG_HOLD, PIDControlSubsystem(), "flightgear")