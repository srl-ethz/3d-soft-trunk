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
from utils import *

rospy.init_node("test_SOFA_Qualysis_integration")
rate = rospy.Rate(50)  # 50hz
marker_array_pub = rospy.Publisher("/sopra/marker_array", MarkerArray, queue_size=1)

num_segments = 3

st_params = SoftTrunkParameters()
st_params.finalize()
cc = CurvatureCalculator(st_params, CurvatureCalculator.SensorType.qualisys, "", 5)

# global const
X_BI_init_nominal = RigidTransform(np.array([0, 0, 0.118]))
X_IT_init_nominal = RigidTransform(np.array([0, 0, 0.118]))

# global variable?
X_BI_init_meas = RigidTransform()
X_IT_init_meas = RigidTransform()

marker_array_msg = MarkerArray()


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


# sleep until data is received from Qualisys (temporary hack solution)
sleep(2)

H_WB = RigidTransform(cc.get_frame(0))
H_WI = RigidTransform(cc.get_frame(1))
H_WT = RigidTransform(cc.get_frame(num_segments - 1))
set_init_params(H_WB, H_WI, H_WT)

# write zeros at the beginning
pressures_array = get_pressures_array()  # default
angles_array = np.ones(4) * eps
write_data_to_file(angles_array, pressures_array)

while not rospy.is_shutdown():
    H_WB = RigidTransform(cc.get_frame(0))
    H_WI = RigidTransform(cc.get_frame(1))
    H_WT = RigidTransform(cc.get_frame(num_segments - 1))
    X_BI, X_IT = calc_rectified_poses(H_WB, H_WI, H_WT)

    pressures_array = get_pressures_array()
    angles_array = poses_to_angles_array(X_BI, X_IT)
    write_data_to_file(angles_array, pressures_array)

    rate.sleep()
