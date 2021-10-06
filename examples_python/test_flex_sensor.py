from softtrunk_pybind_module import CurvatureCalculator, SoftTrunkParameters
from mobilerack_pybind_module import ValveController, QualisysClient

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
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PointStamped,
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
    Twist,
)
from visualization_msgs.msg import MarkerArray, Marker
from festo_phand_msgs.msg import SimpleFluidPressures, GenericSensor, HandState


def xform_rot_dist(x1, x2):
    """[summary]

    Args:
        x1 ([type]): [description]
        x2 ([type]): [description]

    Returns:
        [type]: [description]
    """
    return quat_dist(get_quat(x1), get_quat(x2))


def xform_dist(x1, x2):
    """[summary]

    Args:
        x1 ([type]): [description]
        x2 ([type]): [description]

    Returns:
        [type]: [description]
    """
    return pos_dist(x1.translation(), x2.translation()) + quat_dist(
        get_quat(x1), get_quat(x2)
    )


def quat_dist(q1, q2):
    """[summary]

    Args:
        q1 ([type]): [description]
        q2 ([type]): [description]

    Returns:
        [type]: [description]
    """
    q1_dot_q2 = np.dot(q1.wxyz(), q2.wxyz())
    return np.sqrt(np.power(1 - q1_dot_q2 * q1_dot_q2, 2))


def pos_dist(p1, p2):
    """[summary]

    Args:
        p1 ([type]): [description]
        p2 ([type]): [description]

    Returns:
        [type]: [description]
    """
    return np.linalg.norm(p1 - p2)


def get_rpy(xform):
    """[summary]

    Args:
        xform ([type]): [description]

    Returns:
        [type]: [description]
    """
    return RollPitchYaw(xform.rotation()).vector()


def get_quat(xform):
    """[summary]

    Args:
        xform ([type]): [description]

    Returns:
        [type]: [description]
    """
    return xform.rotation().ToQuaternion()


def xform_to_xyzrpy(xform):
    """[summary]

    Args:
        xform ([type]): [description]

    Returns:
        [type]: [description]
    """
    xyzrpy = np.zeros(6)
    xyzrpy[:3] = xform.translation()
    xyzrpy[:3] *= 1000  # m to mm
    xyzrpy[3:] = get_rpy(xform)
    xyzrpy[3:] *= 180 / np.pi  # rad to deg
    return xyzrpy


def rgb_(value):
    """Converts a value in range [0, 1] to rgb color in [0, 1]^3

    Args:
        value ([float]): [0, 1]

    Returns:
        [numpy.array[float]]: [0, 1]^3
    """
    minimum, maximum = 0.0, 1.0
    ratio = 2 * (value - minimum) / (maximum - minimum)
    b = 1 - ratio
    b = np.clip(b, 0, 1)

    r = ratio - 1
    r = np.clip(r, 0, 1)

    g = 1.0 - b - r
    return np.stack([r, g, b]).transpose()


def sleep(seconds):
    """[summary]

    Args:
        seconds ([type]): [description]
    """
    for ii in range(math.floor(seconds)):
        time.sleep(1)
        print(".", end="", flush=True)
    time.sleep(seconds % 1)
    print("-")


rospy.init_node("test_flex_sensor")
rate = rospy.Rate(50)  # 50hz
marker_array_pub = rospy.Publisher("/sopra/marker_array", MarkerArray, queue_size=1)

num_segments = 3
eps = 1e-10

st_params = SoftTrunkParameters()
st_params.finalize()
cc = CurvatureCalculator(st_params, CurvatureCalculator.SensorType.qualisys, "", 5)
marker_array_msg = MarkerArray()


def handle_flex_sensor_msg(flex_sensor_msg):
    print(f"Flex: {flex_sensor_msg.raw_values[2:5]}")


def handle_hand_state_msg(hand_state_msg):
    print(f"MeasPressures: {hand_state_msg.internal_sensors.actual_pressures}")


set_pressures_topic = "/festo/phand/set_pressures"
flex_sensors_topic = "/festo/phand/connected_sensors/flex_sensors"
hand_state_topic = "/festo/phand/state"

set_pressure_pub = rospy.Publisher(set_pressures_topic, SimpleFluidPressures, queue_size=1)
flex_sensors_sub = rospy.Subscriber(flex_sensors_topic, GenericSensor, handle_flex_sensor_msg)
hand_state_sub = rospy.Subscriber(hand_state_topic, HandState, handle_hand_state_msg)


def init_marker_array():
    global marker_array_msg
    global marker_array_pub
    marker_idx = 0
    for i in range(3):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = marker_idx
        marker_idx = marker_idx + 1
        marker.type = 2
        marker.action = 2
        marker.pose = Pose()
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.frame_locked = False
        marker.ns = "marker-%u" % i
        marker_array_msg.markers.append(marker)
    marker_array_pub.publish(marker_array_msg)


def poses_to_angles_array(X_BI, X_IT):
    """[summary]

    Args:
        X_BI ([type]): [description]
        X_IT ([type]): [description]

    Returns:
        [type]: [description]
    """
    xyzrpy_BI = xform_to_xyzrpy(X_BI)
    xyzrpy_IT = xform_to_xyzrpy(X_IT)

    # invert roll?
    xyzrpy_BI[3] *= -1
    xyzrpy_IT[3] *= -1

    # print(f"X_BI = {xyzrpy_BI}\n\tX_IT = {xyzrpy_IT}")
    return np.hstack([xyzrpy_BI[-3:], xyzrpy_IT[-3:]])


def write_data_to_file(
    angles, pressures, filepath="../../sopra-fem/sensor_test/Temp/AngleData.txt"
):
    """Save angle and pressure data to txt file

    Args:
        angles (4x1 float np.array): 2* 2 angles for each segment
        pressures (6x1 float np.array): 6 chambers
        filepath (str, optional): Defaults to "../../sopra-fem/sensor_test/Temp/AngleData.txt"
    """
    # eps = 1e-10
    save_data_array = np.hstack([angles, pressures])
    np.savetxt(filepath, save_data_array)


# sleep until data is received from Qualisys (temporary hack solution)
sleep(2)

X_WB_init = RigidTransform(cc.get_frame(50))
X_WT_init = RigidTransform(cc.get_frame(51))

while not rospy.is_shutdown():
    X_WB = RigidTransform(cc.get_frame(50))
    X_WT = RigidTransform(cc.get_frame(51))
    X_BT = X_WB.inverse() *  X_WT
    xyzrpy_BT = xform_to_xyzrpy(X_BT)

    print(xyzrpy_BT)

    rate.sleep()
