from softtrunk_pybind_module import CurvatureCalculator, SoftTrunkParameters
from mobilerack_pybind_module import ValveController, QualisysClient
import time


max_pressure = 500
pressure = 300
num_segments = 3
eps = 1e-10

valves = [15]
vc = ValveController("192.168.0.100", valves, max_pressure)

time.sleep(1)

vc.setSinglePressure(0, pressure)

# wait to settle
time.sleep(1)

input()