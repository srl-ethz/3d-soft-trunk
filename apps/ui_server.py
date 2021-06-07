#!/usr/bin/env python
import rospy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool

"""
Server for the Interactive Marker that is used as GUI to designate desired hand position.
publishes to rostopics:
osc_target (JointTrajectoryPoint): marker position & estimated velocity
control_hand (bool): control command to open or close hand. true=open, false=closed

to start,
- start `roslaunch rviz.launch` in urdf/ directory. rviz opens.
- in rviz, File -> Open Config (Recent Configs) -> select robot.rviz in urdf/
- start this program, `./ui_server.py`. Interactive marker should show up in rviz.
- start program which uses VisualizerROS, e.g. ui_controller. robot should show up in rviz.

to use,
- move green ball around to move target position
- right click on green ball to open/close hand.

since this project isn't set up as a ROS package, it's not as simple to launch the ROS-related programs than if it were

http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls
"""

# construct empty messages for position & orientation messages
jtp_msg = JointTrajectoryPoint()
jtp_msg.positions = [0]*3
jtp_msg.velocities = [0]*3
jtp_msg.accelerations = [0]*3
last_msg_time = 0.

def processFeedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        global last_msg_time
        delta_t = rospy.get_time() - last_msg_time
        last_msg_time = rospy.get_time()
        if delta_t < 0.1:
            # calculate velocity only when there is recent old message
            jtp_msg.velocities[0] = (
                feedback.pose.position.x - jtp_msg.positions[0])/delta_t
            jtp_msg.velocities[1] = (
                feedback.pose.position.y - jtp_msg.positions[1])/delta_t
            jtp_msg.velocities[2] = (
                feedback.pose.position.z - jtp_msg.positions[2])/delta_t
        else:
            jtp_msg.velocities = [0] * 3
        jtp_msg.positions = [feedback.pose.position.x,
                             feedback.pose.position.y, feedback.pose.position.z]
        pub.publish(jtp_msg)

    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        if feedback.menu_entry_id == 1:
            rospy.loginfo("Opening hand")
            pub_hand.publish(True)
        elif feedback.menu_entry_id == 2:
            rospy.loginfo("Closing hand")
            pub_hand.publish(False)
    server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("ui_server")
    pub = rospy.Publisher("osc_target", JointTrajectoryPoint, queue_size=10)
    pub_hand = rospy.Publisher("control_hand", Bool, queue_size=10)

    server = InteractiveMarkerServer("softtrunk_interactive_marker")
    menu_handler = MenuHandler()
    menu_handler.insert("Open Hand", callback=processFeedback)
    menu_handler.insert("Close Hand", callback=processFeedback)

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "osc_target_marker"
    int_marker.description = "OSC target"
    int_marker.scale = 0.2

    # set initial position to be easily reachable
    int_marker.pose.position.x = 0.1
    int_marker.pose.position.y = 0.
    int_marker.pose.position.z = -0.25

    sphere_marker = Marker()
    sphere_marker.type = Marker.SPHERE
    sphere_marker.scale.x = 0.03
    sphere_marker.scale.y = 0.03
    sphere_marker.scale.z = 0.03
    sphere_marker.color.r = 0.5
    sphere_marker.color.g = 1.
    sphere_marker.color.b = 0.5
    sphere_marker.color.a = 0.6

    pos_control = InteractiveMarkerControl()
    pos_control.always_visible = True
    pos_control.markers.append(sphere_marker)
    pos_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
    pos_control.name = "position_goal"

    menu_control = InteractiveMarkerControl()
    menu_control.interaction_mode = InteractiveMarkerControl.MENU
    menu_control.name = "menu"
    menu_control.description = "_"

    int_marker.controls = [pos_control, menu_control]
    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)
    server.applyChanges()
    rospy.spin()
