from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.common.eigen_geometry import Quaternion

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
from utils import *


rospy.init_node("publish_poses")
marker_array_topic = "/qualysis/rigid_body_markers"
marker_array_pub = rospy.Publisher(marker_array_topic, MarkerArray, queue_size=10)

def set_marker_pose(idx, X_W_body):
    global marker_array_msg
    body_marker = marker_array_msg.markers[idx]
    body_marker.pose = xform_to_pose(X_W_body)


def publish_marker_array(body_id_to_pose):
    marker_array_msg = MarkerArray()
    marker_idx = 0
    for body_id, pose in body_id_to_pose.items():
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = body_id
        marker_idx = marker_idx + 1
        marker.type = 2
        marker.action = 2
        marker.pose = xform_to_pose(pose)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.frame_locked = False
        marker.ns = "marker-%u" % marker_idx
        marker_array_msg.markers.append(marker)
    marker_array_pub.publish(marker_array_msg)

# sleep until data is received from Qualisys (temporary hack solution)
sleep(2)

body_ids = [0, 1]
body_id_to_pose = {body_id:RigidTransform for body_id in body_ids}

st_params = SoftTrunkParameters()
st_params.finalize()
cc = CurvatureCalculator(st_params, CurvatureCalculator.SensorType.qualisys, "", 500)

rate = rospy.Rate(50)  # 50hz
while not rospy.is_shutdown():

    for idx, body_id in enumerate(body_ids):
        body_id_to_pose[body_id] = RigidTransform(cc.get_frame(body_id))
        # print(f"idx: {idx}\tID: {body_id}\tpose: {body_id_to_pose[body_id]}")
    publish_marker_array(body_id_to_pose)
    rate.sleep()
