from .rp_flight_pid import RPPIDControlSubsystem
from .rp_flight_evals import RPFlightControlEval
from .ha_flight_evals import *
from .ha_flight_pid import *
import numpy as np

basic_flight_evaluator = RPFlightControlEval(custom_tracking_vars=[
    prp.propeller_rpm
])
control_subsystem = RPPIDControlSubsystem()
basic_flight_evaluator.run_specific_eval(0, 1, 0, 0, control_subsystem)
for k, val in basic_flight_evaluator.initial_conditions[-1].items():
    print(f"{k} : {val}")
print("============EVALUATIONS=============")
for k, val in basic_flight_evaluator.evals[-1].items():
    print(f"{k} : {val}")
basic_flight_evaluator.plot_eval(-1)

# Plot projections
import matplotlib.pyplot as plt
time = basic_flight_evaluator.trjs_time[-1]
plt.plot(time, np.array(control_subsystem.pitch_subsystem.projected_pitches) / pi * 180)
plt.show()

