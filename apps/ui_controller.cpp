#include <3d-soft-trunk/VisualizerROS.h>
#include "3d-soft-trunk/OSC.h"
#include <3d-soft-trunk/SoftTrunkModel.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Bool.h>

// see ui_server.py for how to use UI-based operational space control
// this program receives reference positions via ROS from the ui_server.py and relays it to the OSC program

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ui_controller");

    OSC osc(CurvatureCalculator::SensorType::simulator, true);
    SoftTrunkModel stm{};
    VisualizerROS vis{stm};
    srl::State state;
    // VectorXd p;

    // set initial state
    state.q = 0.1 * VectorXd::Ones(st_params::q_size);
    osc.set_state(state);

    ros::NodeHandle nh;
    ros::Subscriber osc_target_sub = nh.subscribe("osc_target", 1, osc_target_cb);

    double hz = 50.;
    srl::Rate r{hz};

    for (double t = 0; t < 60; t += 1./hz)
    {
        r.sleep();
        ros::spinOnce();
        // update state of model used in ROS visualization
        osc.get_state(state);
        stm.updateState(state);
        vis.publishState();
        
        if (is_goal_received)
            osc.set_ref(osc_target, osc_target_d);
        // osc.get_pressure(p);
        // fmt::print("p:{}\n", p.transpose());
        if (!ros::ok())
            break;
    }
}