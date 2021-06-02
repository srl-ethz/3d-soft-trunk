#include <3d-soft-trunk/VisualizerROS.h>


VisualizerROS::VisualizerROS(SoftTrunkModel& stm) : stm(stm){
    joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    joint_msg.name.resize(7 * st_params::num_segments * (st_params::sections_per_segment + 1));
    joint_msg.position.resize(joint_msg.name.size());
    // set up the name of each joint in the rigid body model
    for (int i = 0; i < st_params::num_segments*(st_params::sections_per_segment+1); i++)
    {
        int segment_id = i / (st_params::sections_per_segment+1);
        int section_id = i % (st_params::sections_per_segment+1);

        std::string pcc_name = fmt::format("seg{}_sec{}", segment_id, section_id);
        joint_msg.name[7*i+0] = fmt::format("{}-ball-ball-joint_x_joint", pcc_name);
        joint_msg.name[7*i+1] = fmt::format("{}-ball-ball-joint_y_joint", pcc_name);
        joint_msg.name[7*i+2] = fmt::format("{}-ball-ball-joint_z_joint", pcc_name);
        joint_msg.name[7*i+3] = fmt::format("{}-a-b_joint", pcc_name);
        joint_msg.name[7*i+4] = fmt::format("{}-b-branch-a_joint", pcc_name);
        joint_msg.name[7*i+5] = fmt::format("{}-b-seg{}_sec{}-{}_connect_joint", pcc_name, segment_id, section_id, section_id+1);
        joint_msg.name[7*i+6] = fmt::format("{}-branch-a-branch-b_joint", pcc_name);
    }

    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::ARROW;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.points.resize(2);
    marker_msg.points[0].x = 0; marker_msg.points[0].y=0; marker_msg.points[0].z = 0;
    marker_msg.scale.x = 0.005; marker_msg.scale.y = 0.01;
    marker_msg.color.a = 1;
}

void VisualizerROS::publishState(){
    for (int i = 0; i < joint_msg.name.size(); i++)
        joint_msg.position[i] = stm.ara->xi_(i);
    joint_msg.header.stamp = ros::Time::now();
    joint_pub.publish(joint_msg);
}

void VisualizerROS::publishArrow(int segment, Vector3d xyz, Vector3d rgb, bool global){
    if (global){
        // rotate xyz from global frame to local frame, since Visualization marker assumes local frame
        Matrix3d base_to_segment = stm.ara->get_H(segment).matrix().block(0,0,3,3);
        Matrix3d world_to_base = stm.ara->get_H_base().matrix().block(0,0,3,3);
        xyz = base_to_segment.inverse() * world_to_base.inverse() * xyz;
    }
       
    marker_msg.header.frame_id = fmt::format("seg{}_sec{}-{}_connect", st_params::num_segments-1, st_params::sections_per_segment, st_params::sections_per_segment+1);
    marker_msg.points[1].x = xyz(0); marker_msg.points[1].y = xyz(1); marker_msg.points[1].z = xyz(2); 
    marker_msg.color.r = rgb(0); marker_msg.color.g = rgb(1); marker_msg.color.b = rgb(2); 
    marker_msg.header.stamp = ros::Time::now();
    marker_pub.publish(marker_msg);
}