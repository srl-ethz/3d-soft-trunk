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
import visdom
import threading
import datetime

rospy.init_node("test_flex_sensor")
rate = rospy.Rate(50)  # 50hz
marker_array_pub = rospy.Publisher("/sopra/marker_array", MarkerArray, queue_size=1)

num_segments = 3
plot_labels = [
    "Time (s)",
    "Commanded Pressure #1 (kPa)",
    "Commanded Pressure #2 (kPa)",
    "Commanded Pressure #3 (kPa)",
    "Measured Pressure #1 (kPa)",
    "Measured Pressure #2 (kPa)",
    "Measured Pressure #3 (kPa)",
    "Flex sensor #1",
    "Flex sensor #2",
    "Flex sensor #3",
    "Quaternion dist from MoCap (rad) * 400 + 1000",
]

data_cols = [
    "time",
    "cmd_P1",
    "cmd_P2",
    "cmd_P3",
    "meas_P1",
    "meas_P2",
    "meas_P3",
    "flex1",
    "flex2",
    "flex3",
    "q_dist",
    "X_BT_meas_x",
    "X_BT_meas_y",
    "X_BT_meas_z",
    "X_BT_meas_qw",
    "X_BT_meas_qx",
    "X_BT_meas_qy",
    "X_BT_meas_qz",
]

vis = visdom.Visdom()
plot_opts = dict(
    # title="",
    xlabel=plot_labels[0],
    ylabel="Sensor readings",
    legend=plot_labels[1:],
)
plot_start_time = None
plot = None
output_data_list = []
dataframe_lock = threading.Lock()

# st_params = SoftTrunkParameters()
# st_params.finalize()
# cc = CurvatureCalculator(st_params, CurvatureCalculator.SensorType.qualisys, "", 5)
marker_array_msg = MarkerArray()
flex_sensor_reading = None
pressure_reading = None
cmd_pressure_values = None
X_BT_init = None
X_BT_meas = None

# flags
updated_pose = threading.Event()
updated_pressure = threading.Event()
updated_sensor_reading = threading.Event()


def handle_flex_sensor_msg(flex_sensor_msg):
    global updated_sensor_reading
    global flex_sensor_reading
    if updated_sensor_reading.is_set():
        return
    flex_sensor_reading = flex_sensor_msg.raw_values[2:5]
    updated_sensor_reading.set()
    # print(f"Flex: {flex_sensor_reading}")


def handle_hand_state_msg(hand_state_msg):
    global updated_pressure
    global pressure_reading
    global cmd_pressure_values
    if updated_pressure.is_set():
        return
    pressure_reading = hand_state_msg.internal_sensors.actual_pressures.values[:3]
    cmd_pressure_values = hand_state_msg.internal_sensors.set_pressures.values[:3]
    updated_pressure.set()
    # print(f"MeasPressures: {pressure_reading}")


# finger base id
B_id = 0

# finger tip id
T_id = 1


def handle_marker_array(marker_array_msg):
    global updated_pose
    global X_BT_meas
    if updated_pose.is_set():
        return
    body_id_to_pose = dict()
    for marker in marker_array_msg.markers:
        body_id = marker.id
        xform = pose_to_xform(marker.pose)
        body_id_to_pose[body_id] = xform
    if B_id in body_id_to_pose and T_id in body_id_to_pose:
        X_BT_meas = body_id_to_pose[B_id].inverse() @ body_id_to_pose[T_id]
        updated_pose.set()


def clear_flags():
    global updated_pose
    global updated_pressure
    global updated_sensor_reading
    updated_pose.clear()
    updated_pressure.clear()
    updated_sensor_reading.clear()


def plot_loop():
    global updated_pose
    global updated_pressure
    global updated_sensor_reading

    while True:
        while not (
            updated_pose.is_set()
            and updated_pressure.is_set()
            and updated_sensor_reading.is_set()
        ):
            time.sleep(0.01)

        global vis
        global plot
        global plot_opts
        global plot_start_time

        time_now = time.time()
        qdist = quat_dist(get_quat(X_BT_meas), Quaternion())
        sensor_readings = np.hstack(
            [cmd_pressure_values, pressure_reading, flex_sensor_reading, [qdist], xform_to_xyzquat(X_BT_meas)]
        )
        sensor_readings[:6] /= 1e3  # to kPa

        plot_data = np.copy(sensor_readings[:10])
        plot_data[-1] = 1000 + plot_data[-1] * 400
        plot_data = plot_data.reshape((-1, 1))
        if plot is None:
            plot_start_time = time_now

        if plot is not None and vis.win_exists(plot):
            vis.line(
                Y=plot_data,
                X=[time_now - plot_start_time],
                win=plot,
                opts=plot_opts,
                update="append",
            )  # name="regression"
        else:
            plot = vis.line(
                Y=plot_data,
                X=np.array([time_now - plot_start_time]),
                opts=plot_opts,
            )
        with dataframe_lock:
            output_data_list.append(
                [time_now - plot_start_time] + sensor_readings.tolist()
            )
        clear_flags()


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
sleep_to_settle_time = 0.5

# set everything to zero and measure initial pose
pressures = [1e5] * num_valves
set_pressures(pressures)
sleep(sleep_to_settle_time)  # HACK
# measure once it settles

data_list = []
max_num_iter = 10

plot_thread = threading.Thread(target=plot_loop)
plot_thread.start()

time_start = datetime.datetime.now()
datetime_str = time_start.strftime("%Y%m%d%H%M%S")

finger_idx = 2
for iter_idx in trange(max_num_iter):
    cmd_pressures = np.arange(1e5, 4.5e5, 5e4).tolist()
    for dir in tqdm(["up", "down"], leave=False):  # ramp up and down
        for pressure_desired in tqdm(cmd_pressures, leave=False):
            desired_pressures = [1e5] * num_valves
            desired_pressures[finger_idx] = pressure_desired
            set_pressures(desired_pressures)
            # measure once it settles
            sleep(sleep_to_settle_time)

        pressures = [1e5] * num_valves
        set_pressures(pressures)
        sleep(sleep_to_settle_time)

        cmd_pressures = cmd_pressures[::-1]

        with dataframe_lock:
            df = pd.DataFrame(
                output_data_list,
                columns=data_cols,
            )
            csv_fpath = f"/src/3d-soft-trunk/logs/{datetime_str}-p1to45bar-{dir}-wait{sleep_to_settle_time}secs-f{finger_idx}-i{iter_idx}.csv"
            df.to_csv(csv_fpath)
            output_data_list = []
            print(f"Saved to {csv_fpath}")
