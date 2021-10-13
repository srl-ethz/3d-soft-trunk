# from softtrunk_pybind_module import CurvatureCalculator, SoftTrunkParameters
# from mobilerack_pybind_module import ValveController, QualisysClient

import math
import time
import numpy as np
import matplotlib.pyplot as plt

from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.common.eigen_geometry import Quaternion
from collections import defaultdict

np.set_printoptions(precision=4)
plt.rcParams["savefig.facecolor"] = "0.8"

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PointStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from visualization_msgs.msg import MarkerArray, Marker
from festo_phand_msgs.msg import SimpleFluidPressures, GenericSensor, HandState
from utils import *

import pandas as pd
from tqdm import tqdm, trange

rospy.init_node("test_flex_sensor")
rate = rospy.Rate(50)  # 50hz
marker_array_pub = rospy.Publisher("/sopra/marker_array", MarkerArray, queue_size=1)

num_segments = 3

# st_params = SoftTrunkParameters()
# st_params.finalize()
# cc = CurvatureCalculator(st_params, CurvatureCalculator.SensorType.qualisys, "", 5)
marker_array_msg = MarkerArray()
flex_sensor_reading = None
pressure_reading = None
X_BT_init = None
X_BT_meas = None

# flags
updated_pose = True
updated_pressure = True
updated_sensor_reading = True


def handle_flex_sensor_msg(flex_sensor_msg):
    global updated_sensor_reading
    global flex_sensor_reading
    if updated_sensor_reading:
        return
    flex_sensor_reading = flex_sensor_msg.raw_values[2:5]
    updated_sensor_reading = True
    # print(f"Flex: {flex_sensor_reading}")


def handle_hand_state_msg(hand_state_msg):
    global updated_pressure
    global pressure_reading
    if updated_pressure:
        return
    pressure_reading = hand_state_msg.internal_sensors.actual_pressures.values[:3]
    updated_pressure = True
    # print(f"MeasPressures: {pressure_reading}")


# finger base id
B_id = 8

# finger tip id
T_id = 7


def handle_marker_array(marker_array_msg):
    global updated_pose
    global X_BT_meas
    if updated_pose:
        return
    body_id_to_pose = dict()
    for marker in marker_array_msg.markers:
        body_id = marker.id
        xform = pose_to_xform(marker.pose)
        body_id_to_pose[body_id] = xform
    if B_id in body_id_to_pose and T_id in body_id_to_pose:
        X_BT_meas = body_id_to_pose[B_id].inverse() @ body_id_to_pose[T_id]
        updated_pose = True


def clear_flags():
    global updated_pose
    global updated_pressure
    global updated_sensor_reading
    updated_pose = False
    updated_pressure = False
    updated_sensor_reading = False


def wait_for_measurements():
    global updated_pose
    global updated_pressure
    global updated_sensor_reading

    while not (updated_pose and updated_pressure and updated_sensor_reading):
        time.sleep(0.1)


set_pressures_topic = "/festo/phand/set_pressures"
flex_sensors_topic = "/festo/phand/connected_sensors/flex_sensors"
hand_state_topic = "/festo/phand/state"
marker_array_topic = "/qualysis/rigid_body_markers"


set_pressure_pub = rospy.Publisher(
    set_pressures_topic, SimpleFluidPressures, queue_size=1
)
flex_sensors_sub = rospy.Subscriber(
    flex_sensors_topic, GenericSensor, handle_flex_sensor_msg
)
hand_state_sub = rospy.Subscriber(hand_state_topic, HandState, handle_hand_state_msg)
marker_array_sub = rospy.Subscriber(
    marker_array_topic, MarkerArray, handle_marker_array
)

num_valves = 3


def set_pressures(pressures):
    msg = SimpleFluidPressures()
    msg.values = pressures
    set_pressure_pub.publish(msg)


# sleep until data is received from Qualisys (temporary hack solution)
sleep(2)

finger_idx = 2
sleep_to_settle_time = 5

# set everything to zero and measure initial pose
pressures = [0] * num_valves
set_pressures(pressures)
sleep(sleep_to_settle_time)  # HACK
# measure once it settles
print("Measuring X_BT initial...")
clear_flags()
wait_for_measurements()
X_BT_init = X_BT_meas

# data_cols = [
#     "finger_idx",
#     "P_desired",
#     "P_measured",
#     "X_BT_meas.x",
#     "X_BT_meas.y",
#     "X_BT_meas.z",
#     "X_BT_meas.qw",
#     "X_BT_meas.qx",
#     "X_BT_meas.qy",
#     "X_BT_meas.qz",
#     "quat_dist_BT_meas",
#     "flex_sensor",
# ]
data_list = []
max_num_iter = 50


def data_to_dict(finger_idx, state, Pdesired, Pmeas, X_BT_meas, flex_meas):
    xyzq = xform_to_xyzquat(X_BT_meas)
    return {
        "finger_idx": finger_idx,
        "P_desired": Pdesired,
        "P_measured": Pmeas,
        "X_BT_meas.xyz_qwxyz": xyzq,
        "quat_dist_BT_meas": quat_dist(get_quat(X_BT_meas), Quaternion()),
        "flex_sensor": flex_meas,
    }
csv_fpath = f"/src/3d-soft-trunk/examples_python/20211007-f{finger_idx}-p0to6bar-n{max_num_iter}-v1-wait5secs-test2.csv"

for _ in trange(max_num_iter):
    for pressure_desired in tqdm([0, 3e5, 4e5, 5e5, 6e5], leave=False):
        desired_pressures = [0] * num_valves
        desired_pressures[finger_idx] = pressure_desired
        set_pressures(desired_pressures)
        # measure once it settles
        sleep(sleep_to_settle_time)
        clear_flags()
        wait_for_measurements()
        # print(
        #     f"@idx:{finger_idx} Pd:{desired_pressures[:3]} P:{pressure_reading} X_BT_meas:{X_BT_meas} q: {quat_dist(get_quat(X_BT_meas), Quaternion())} flex:{flex_sensor_reading}"
        # )
        data_list.append(
            data_to_dict(
                finger_idx,
                "closed" if pressure_desired > 1e5 else "open",
                desired_pressures[:3],
                pressure_reading,
                X_BT_meas,
                flex_sensor_reading,
            )
        )
    df = pd.DataFrame(data_list)
    df.to_csv(csv_fpath)
print(f"Saved to {csv_fpath}")

pressures = [0] * num_valves
set_pressures(pressures)