#include <3d-soft-trunk/VisualizerROS.h>
#include "3d-soft-trunk/OSC.h"
#include <3d-soft-trunk/SoftTrunkModel.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Bool.h>

// see ui_server.py for how to use UI-based operational space control
// this program receives reference positions via ROS from the ui_server.py and relays it to the OSC program
// currently is a bit unstable....

Vector3d osc_target;
Vector3d osc_target_d;
bool is_goal_received = false;

void osc_target_cb(const trajectory_msgs::JointTrajectoryPoint &msg)
{
    for (int i = 0; i < 3; i++)
    {
        osc_target(i) = msg.positions[i];
        osc_target_d(i) = msg.velocities[i];
    }
    if (!is_goal_received)
        is_goal_received = true;
}

void control_hand_cb(const std_msgs::Bool &msg){
    /** @todo implement code to actually actuate hand */
    if (msg.data)
        ROS_INFO("open hand");
    else
        ROS_INFO("close hand");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ui_controller");
    SoftTrunkParameters st_params{};
    st_params.finalize();

    OSC osc(st_params, CurvatureCalculator::SensorType::simulator, true);

    // for ROS-based visualization
    SoftTrunkModel stm{st_params};
    VisualizerROS vis{stm};
    srl::State state = st_params.getBlankState();

    // subscribe to ros topic
    ros::NodeHandle nh;
    ros::Subscriber osc_target_sub = nh.subscribe("osc_target", 1, osc_target_cb);
    ros::Subscriber control_hand_sub = nh.subscribe("control_hand", 1, control_hand_cb);

    // set initial state
    state.q = 0.1 * VectorXd::Ones(st_params.q_size);
    osc.set_state(state);

    double hz = 50.; // hardcoded in OSC.cpp?
    srl::Rate r{hz};

    while (true)
    {
        r.sleep();
        ros::spinOnce();  // receive messages from ROS
        // update state of model used in ROS visualization
        osc.get_state(state);
        stm.updateState(state);
        vis.publishState();
        
        if (is_goal_received)
            // osc.set_ref(osc_target, osc_target_d);
        if (!ros::ok())
            break;
    }
}
