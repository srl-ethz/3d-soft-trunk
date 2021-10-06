from softtrunk_pybind_module import CurvatureCalculator, SoftTrunkParameters
from mobilerack_pybind_module import ValveController, QualisysClient

import math
import time
import numpy as np
import matplotlib.pyplot as plt

from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.common.eigen_geometry import Quaternion
from collections import defaultdict
from utils import *

np.set_printoptions(precision=4)
plt.rcParams["savefig.facecolor"] = "0.8"

num_segments = 3
# valves = [2, 6, 4, 0, 5, 3]
# 6 is diconnected on 2021.10.05
valves = [2, 4, 0, 5, 3]
# list(range(7))
# # valve 1 is the gripper. skip
# valves.remove(1)

max_pressure = 500
pressure = 300

vc = ValveController("192.168.0.100", valves, max_pressure)

st_params = SoftTrunkParameters()
st_params.finalize()
cc = CurvatureCalculator(st_params, CurvatureCalculator.SensorType.qualisys, "", 5)

# global const
X_BI_init_nominal = RigidTransform(np.array([0, 0, 0.118]))
X_IT_init_nominal = RigidTransform(np.array([0, 0, 0.118]))

# global variable?
X_BI_init_meas = RigidTransform()
X_IT_init_meas = RigidTransform()


def set_init_params(H_WB, H_WI, H_WT):
    """[summary]

    Args:
        H_WB ([type]): [description]
        H_WI ([type]): [description]
        H_WT ([type]): [description]
    """
    global X_BI_init_meas
    global X_IT_init_meas
    global X_BI_init_diff
    global X_BI_init_nominal
    global X_IT_init_diff
    global X_IT_init_nominal

    X_WB = RigidTransform(H_WB)
    X_WI = RigidTransform(H_WI)
    X_WT = RigidTransform(H_WT)
    X_BI_init_meas = X_WB.inverse() @ X_WI
    X_IT_init_meas = X_WI.inverse() @ X_WT
    X_BI_init_diff = X_BI_init_nominal.inverse() @ X_BI_init_meas
    X_IT_init_diff = X_IT_init_nominal.inverse() @ X_IT_init_meas
    print(f"X_BI_init_meas = {xform_to_xyzrpy(X_BI_init_meas)}")
    print(f"X_IT_init_meas = {xform_to_xyzrpy(X_IT_init_meas)}")
    print(f"X_BI_init_diff = {xform_to_xyzrpy(X_BI_init_diff)}")
    print(f"X_IT_init_diff = {xform_to_xyzrpy(X_IT_init_diff)}")


def calc_rectified_poses(H_WB, H_WI, H_WT):
    """[summary]

    Args:
        H_WB ([type]): [description]
        H_WI ([type]): [description]
        H_WT ([type]): [description]

    Returns:
        [type]: [description]
    """
    global X_BI_init_diff
    global X_IT_init_diff
    X_WB = RigidTransform(H_WB)
    X_WI = RigidTransform(H_WI)
    X_WT = RigidTransform(H_WT)
    X_BI_meas = X_WB.inverse() @ X_WI
    X_BI_meas = X_BI_init_diff.inverse() @ X_BI_meas
    X_IT_meas = X_WI.inverse() @ X_WT
    X_IT_meas = X_IT_init_diff.inverse() @ X_IT_meas
    return X_BI_meas, X_IT_meas


sleep(1)  # sleep until data is received from Qualisys (temporary hack solution)

q, dq, ddq = cc.get_curvature()
H_WB = RigidTransform(cc.get_frame(0))
H_WI = RigidTransform(cc.get_frame(1))
H_WT = RigidTransform(cc.get_frame(num_segments - 1))
timestamp = cc.get_timestamp()
set_init_params(H_WB, H_WI, H_WT)

pressures_array = get_pressures_array()  # default
angles_array = np.ones(4) * eps
write_data_to_file(angles_array, pressures_array)


# for ii in range(len(valves)):
#     print(f"\nindex:{ii}\tvalve id:{valves[ii]}\tpressure:{pressure}")
#     # set pressure
#     vc.setSinglePressure(ii, pressure)
#     sleep(2)
#     vc.setSinglePressure(ii, 0)
#     sleep(1)
# exit()

for ii in range(len(valves)):
    print(f"\nindex:{ii}\tvalve id:{valves[ii]}\tpressure:{pressure}")
    # set pressure
    vc.setSinglePressure(ii, pressure)

    # wait to settle
    sleep(10)

    q, dq, ddq = cc.get_curvature()
    H_WB = RigidTransform(cc.get_frame(0))
    H_WI = RigidTransform(cc.get_frame(1))
    H_WT = RigidTransform(cc.get_frame(num_segments - 1))
    timestamp = cc.get_timestamp()
    X_BI, X_IT = calc_rectified_poses(H_WB, H_WI, H_WT)

    pressures_array = get_pressures_array(ii, pressure)
    angles_array = poses_to_angles_array(X_BI, X_IT)
    write_data_to_file(angles_array, pressures_array)

    # sleep(5)
    input()

    # reset to zero
    vc.setSinglePressure(ii, 0)
    sleep(1)

# wait to settle
sleep(10)

print("All valves closed")
q, dq, ddq = cc.get_curvature()
H_WB = RigidTransform(cc.get_frame(0))
H_WI = RigidTransform(cc.get_frame(1))
H_WT = RigidTransform(cc.get_frame(num_segments - 1))
timestamp = cc.get_timestamp()
X_BI, X_IT = calc_rectified_poses(H_WB, H_WI, H_WT)

pressures_array = get_pressures_array(ii, pressure)
angles_array = poses_to_angles_array(X_BI, X_IT)
write_data_to_file(angles_array, pressures_array)
