from softtrunk_pybind_module import CurvatureCalculator
from time import sleep
import numpy as np

cc = CurvatureCalculator(CurvatureCalculator.SensorType.qualisys, "")

num_segments = 3

sleep(1) # sleep until data is received from Qualisys (temporary hack solution)
for i in range(10):
    q, dq, ddq = cc.get_curvature()
    H_base = cc.get_frame(0)
    H_tip = cc.get_frame(num_segments)
    timestamp = cc.get_timestamp()
    print(f"t: {timestamp}\tq: {q}")
    print(f"pose of tip (rel. to base): {np.matmul(np.linalg.inv(H_base), H_tip)}")
    sleep(0.5)