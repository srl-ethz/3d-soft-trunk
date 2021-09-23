#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include "3d-soft-trunk/SoftTrunkModel.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>


/**
 * @brief publish visualization of robot and markers to ROS. (Obviously, ROS must be installed to use this)
 * 
 * It utilizes the URDF file generated for calculating the rigid body parameters, and publishes the sensor_msgs/JointState rosmessages for that robot model.
 * Therefore an rviz instance which loads that robot model URDF file can show the current robot state configuration. 
 * 
 * If using this in your program, set Cmake command so that it will be skipped for non-ROS machines.
 * ```cmake
 * if(${roscpp_FOUND})
 *  add_executable(your_program your_program.cpp)
 *  target_link_libraries(your_program SoftTrunkModel VisualizerROS)
 *  target_include_directories(your_program PRIVATE ${roscpp_INCLUDE_DIRS})    
 * endif(${roscpp_FOUND})
 * ```
 * To show visualization, 
 * ```bash
 * roscore # start ros master
 * ./bin/example_VisualizerROS
 * roslaunch rviz.launch # in 3d-soft-trunk/urdf/
 * # load 3d-soft-trunk/urdf/robot.rviz configuration file into rviz with *File -> Open Config*
 * ```
 */
class VisualizerROS{
    const SoftTrunkModel& stm;
    const SoftTrunkParameters st_params;

    ros::NodeHandle nh;
    ros::Publisher joint_pub;
    ros::Publisher marker_pub;

    sensor_msgs::JointState joint_msg;
    visualization_msgs::Marker marker_msg;

public:
    /**
     * @param stm reference to SoftTrunkModel which this instance visualizes
     */
    VisualizerROS(SoftTrunkModel& stm);
    /**
     * @brief publish the current joint state of SoftTrunkModel.
     */
    void publishState();
    /**
     * @brief publish an arrow marker to Rviz
     * 
     * @param segment ID of segment in which marker is attached (at the tip of the segment)
     * @param xyz position of tip of arrow, in meters
     * @param rgb rgb as value between 0 - 1.
     * @param global xyz relative to global frame or not
     */
    void publishArrow(int segment, Vector3d xyz, Vector3d rgb, bool global = true);
};