from .ha_flight_evals import *
from .basic_flight_pid import *

basic_flight_evaluator = HAFlightControlEval(custom_tracking_vars=['fcs/throttle-pos-norm',
'fcs/throttle-cmd-norm',
prp.pitch_rad])
basic_flight_evaluator.run_single_eval(AltCase.ALT_HOLD, HdgCase.HDG_HOLD, HAPIDControlSubsystem(), "")
basic_flight_evaluator.plot_eval(-1)
print(basic_flight_evaluator.evals[0])