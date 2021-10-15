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
    Twist,
)
from visualization_msgs.msg import MarkerArray, Marker
from utils import *


# constants
eps = 1e-10


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

    # https://math.stackexchange.com/a/90098

    # distance w/o trig functions
    # return np.sqrt(np.power(1 - q1_dot_q2 * q1_dot_q2, 2))

    # angular distance
    return np.acos(2 * np.power(q1_dot_q2, 2) - 1)


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


def xform_to_xyzquat(xform):
    xyzq = np.zeros(7)
    xyzq[:3] = xform.translation()
    q_wxyz = get_quat(xform)
    xyzq[3] = q_wxyz.w()
    xyzq[4] = q_wxyz.x()
    xyzq[5] = q_wxyz.y()
    xyzq[6] = q_wxyz.z()
    return xyzq


def xform_to_pose(X_AB):
    pose = Pose()
    xyz_AB = X_AB.translation()
    pose.position.x = xyz_AB[0]
    pose.position.y = xyz_AB[1]
    pose.position.z = xyz_AB[2]

    q_wxyz_AB = get_quat(X_AB)
    pose.orientation.w = q_wxyz_AB.w()
    pose.orientation.x = q_wxyz_AB.x()
    pose.orientation.y = q_wxyz_AB.y()
    pose.orientation.z = q_wxyz_AB.z()
    return pose


def pose_to_xform(pose):
    xyz = np.array([pose.position.x, pose.position.y, pose.position.z])
    q_wxyz = Quaternion(w=pose.orientation.w, x=pose.orientation.x, y=pose.orientation.y, z=pose.orientation.z)
    return RigidTransform(quaternion=q_wxyz, p=xyz)


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
    return np.hstack([xyzrpy_BI[-3:-1], xyzrpy_IT[-3:-1]])


def get_pressures_array(idx=None, pressure=None):
    """[summary]

    Args:
        idx ([type], optional): [description]. Defaults to None.
        pressure ([type], optional): [description]. Defaults to eps.
    """
    pressures_array = np.ones(6) * eps
    if idx is not None:
        pressures_array[idx] = pressure
    return pressures_array


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
