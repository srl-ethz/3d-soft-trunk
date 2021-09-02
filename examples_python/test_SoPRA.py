from softtrunk_pybind_module import CurvatureCalculator, SoftTrunkParameters
from mobilerack_pybind_module import ValveController, QualisysClient

from time import sleep

# valve 5 is the gripper. skip
valves = list(range(5)) + [6]
max_pressure = 500
pressure = 300
num_segments = 3

vc = ValveController("192.168.0.100", valves, max_pressure)

st_params = SoftTrunkParameters()
st_params.finalize()
cc = CurvatureCalculator(st_params, CurvatureCalculator.SensorType.qualisys, "")

# skip for now since we're not doing dynamic motions anyways
# _, timestamp = qc.getData6D()
# vc.syncTimeStamp(timestamp//1000)  # sync the time to be that of QTM

sleep(1) # sleep until data is received from Qualisys (temporary hack solution)
for i in range(len(valves)):

    print(f"index:{i}\tvalve id:{valves[i]}\tpressure:{pressure}")
    vc.setSinglePressure(i, pressure)
    sleep(10)

    q, dq, ddq = cc.get_curvature()
    H_base = cc.get_frame(0)
    H_intermediate = cc.get_frame(1)
    H_tip = cc.get_frame(num_segments-1)
    timestamp = cc.get_timestamp()

    print(f"\n\nidx: {i}")
    print(f"q: {q}")
    print(f"H_base: {H_base}")
    print(f"H_intermediate: {H_intermediate}")
    print(f"H_tip: {H_tip}")
    print(f"timestamp: {timestamp}")

    # reset to zero
    vc.setSinglePressure(i, 0)
    sleep(1)