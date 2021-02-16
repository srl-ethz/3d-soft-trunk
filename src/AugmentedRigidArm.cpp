// Copyright 2018 ...
#include "3d-soft-trunk/AugmentedRigidArm.h"

AugmentedRigidArm::AugmentedRigidArm()
{
    setup_drake_model();
}

void AugmentedRigidArm::setup_drake_model()
{
    // load robot model into Drake
    // cf: https://drake.guzhaoyuan.com/thing-to-do-in-drake/create-a-urdf-sdf-robot
    // https://github.com/RobotLocomotion/drake/blob/fc77701531605956fc3f6979c9bd2a156e354039/examples/multibody/cart_pole/cart_pole_passive_simulation.cc
    scene_graph.set_name("scene_graph");
    multibody_plant->set_name(st_params::robot_name);
    multibody_plant->RegisterAsSourceForSceneGraph(&scene_graph);

    std::string urdf_file = fmt::format("{}/urdf/{}.urdf", SOFTTRUNK_PROJECT_DIR, st_params::robot_name);
    fmt::print("loading URDF file {}...\n", urdf_file);
    // load URDF into multibody_plant
    drake::multibody::ModelInstanceIndex plant_model_instance_index = drake::multibody::Parser(multibody_plant,
                                                                                               &scene_graph)
                                                                          .AddModelFromFile(
                                                                              urdf_file);

    // weld base link to world frame
    drake::math::RigidTransform<double> world_to_base{};
    if (st_params::armConfiguration == ArmConfigurationType::stalactite)
        world_to_base.set_rotation(drake::math::RollPitchYaw(PI, 0., 0.));
    multibody_plant->WeldFrames(multibody_plant->world_frame(), multibody_plant->GetFrameByName("base_link"),
                                world_to_base);
    multibody_plant->Finalize();

    // connect plant with scene_graph to get collision info
    builder.Connect(multibody_plant->get_geometry_poses_output_port(),
                    scene_graph.get_source_pose_port(multibody_plant->get_source_id().value()));
    builder.Connect(scene_graph.get_query_output_port(), multibody_plant->get_geometry_query_input_port());

    drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

    diagram = builder.Build();
    diagram_context = diagram->CreateDefaultContext();

    // This is supposed to be required to visualize without simulation, but it does work without one...
    // drake::geometry::DrakeVisualizer::DispatchLoadMessage(scene_graph, lcm);

    int num_joints = multibody_plant->num_joints() - 1; // @todo: but why??
    fmt::print("model has {} joints\n", num_joints);
    joints_per_segment = num_joints / st_params::num_segments;

    // check that parameters make sense, just in case
    assert(st_params::num_segments * 2 - 1 == st_params::lengths.size());
    assert(st_params::num_segments + 1 == st_params::diameters.size());
    assert(joints_per_segment == 5 * st_params::sections_per_segment);

    // initialize variables
    xi = VectorXd::Zero(num_joints);
    B_xi = MatrixXd::Zero(num_joints, num_joints);
    G_xi = VectorXd::Zero(num_joints);
    Jm = MatrixXd::Zero(num_joints, 2 * st_params::num_segments * st_params::sections_per_segment);
    dJm = MatrixXd::Zero(num_joints, 2 * st_params::num_segments * st_params::sections_per_segment);
    Jxi = MatrixXd::Zero(3, num_joints);
    update_drake_model();
}

void AugmentedRigidArm::calculate_m(const VectorXd &q, VectorXd &xi)
{
    xi = VectorXd::Zero(joints_per_segment * st_params::num_segments);
    // intermediate variables useful for calculation
    double p0 = 0;
    double t0 = 0;
    double p1;
    double t1;
    double l;
    int joint_id_head;
    for (int section_id = 0; section_id < st_params::num_segments * st_params::sections_per_segment; ++section_id)
    {
        // use phi, theta parametrization because it's simpler for calculation
        p1 = atan2(q(2 * section_id + 1), q(2 * section_id));
        t1 = sqrt(pow(q(section_id * 2), 2) + pow(q(section_id * 2 + 1), 2));

        joint_id_head = 5 * section_id;
        l = st_params::lengths[2 * section_id] / st_params::sections_per_segment;
        // calculate joint angles that kinematically and dynamically match
        // this was calculated from Mathematica and not by hand
        xi(joint_id_head) = -ArcSin((Cos(t1) * Sin(p0) * Sin(t0) - Cos(p0) * Cos(p1) * Sin(p0) * Sin(t1) + Cos(p0) * Cos(p1) * Cos(t0) * Sin(p0) * Sin(t1) +
                                     Power(Cos(p0), 2) * Sin(p1) * Sin(t1) + Cos(t0) * Sin(p1) * Sin(t1) - Power(Cos(p0), 2) * Cos(t0) * Sin(p1) * Sin(t1)) /
                                    Sqrt(1 - Power(Cos(p0) * Cos(t1) * Sin(t0) + Cos(p1) * Cos(t0) * Sin(t1) + Cos(p1) * Power(Sin(p0), 2) * Sin(t1) -
                                                       Cos(p1) * Cos(t0) * Power(Sin(p0), 2) * Sin(t1) - Cos(p0) * Sin(p0) * Sin(p1) * Sin(t1) + Cos(p0) * Cos(t0) * Sin(p0) * Sin(p1) * Sin(t1),
                                                   2)));
        xi(joint_id_head + 1) = ArcSin(Cos(p0) * Cos(t1) * Sin(t0) + Cos(p1) * Cos(t0) * Sin(t1) + Cos(p1) * Power(Sin(p0), 2) * Sin(t1) - Cos(p1) * Cos(t0) * Power(Sin(p0), 2) * Sin(t1) -
                                       Cos(p0) * Sin(p0) * Sin(p1) * Sin(t1) + Cos(p0) * Cos(t0) * Sin(p0) * Sin(p1) * Sin(t1));
        xi(joint_id_head + 2) = -ArcSin((-(Cos(p0) * Power(Cos(p1), 2) * Sin(p0)) + Cos(p0) * Power(Cos(p1), 2) * Cos(t0) * Sin(p0) - Cos(p0) * Cos(t1) * Sin(p0) +
                                         Cos(p0) * Power(Cos(p1), 2) * Cos(t1) * Sin(p0) + Cos(p0) * Cos(t0) * Cos(t1) * Sin(p0) - Cos(p0) * Power(Cos(p1), 2) * Cos(t0) * Cos(t1) * Sin(p0) -
                                         Cos(p1) * Cos(t0) * Sin(p1) + Cos(p1) * Cos(t0) * Cos(t1) * Sin(p1) - Cos(p1) * Power(Sin(p0), 2) * Sin(p1) +
                                         Cos(p1) * Cos(t0) * Power(Sin(p0), 2) * Sin(p1) + Cos(p1) * Cos(t1) * Power(Sin(p0), 2) * Sin(p1) -
                                         Cos(p1) * Cos(t0) * Cos(t1) * Power(Sin(p0), 2) * Sin(p1) - Cos(p0) * Sin(p1) * Sin(t0) * Sin(t1)) /
                                        Sqrt(1 - Power(Cos(p0) * Cos(t1) * Sin(t0) + Cos(p1) * Cos(t0) * Sin(t1) + Cos(p1) * Power(Sin(p0), 2) * Sin(t1) -
                                                           Cos(p1) * Cos(t0) * Power(Sin(p0), 2) * Sin(t1) - Cos(p0) * Sin(p0) * Sin(p1) * Sin(t1) + Cos(p0) * Cos(t0) * Sin(p0) * Sin(p1) * Sin(t1),
                                                       2)));

        if (t1 == 0)
            xi(joint_id_head + 3) = 0;
        else
            xi(joint_id_head + 3) = l / 2 - l * sin(t1 / 2) / t1;
        xi(joint_id_head + 4) = xi(joint_id_head + 3);
        t0 = t1;
        p0 = p1;
    }
}

void AugmentedRigidArm::update_drake_model()
{
    // update drake model
    drake::systems::Context<double> &plant_context = diagram->GetMutableSubsystemContext(*multibody_plant,
                                                                                         diagram_context.get());
    multibody_plant->SetPositions(&plant_context, xi);

    // update drake visualization
    diagram->Publish(*diagram_context);

    // update some dynamic & kinematic params
    multibody_plant->CalcMassMatrix(plant_context, &B_xi);
    G_xi = multibody_plant->CalcGravityGeneralizedForces(plant_context);

    std::string final_frame_name = fmt::format("seg{}-{}_connect", st_params::num_segments - 1, st_params::num_segments);
    multibody_plant->CalcJacobianTranslationalVelocity(plant_context, drake::multibody::JacobianWrtVariable::kQDot,
                                                       multibody_plant->GetFrameByName(final_frame_name),
                                                       VectorXd::Zero(3), multibody_plant->world_frame(),
                                                       multibody_plant->world_frame(), &Jxi);
    H_tip = multibody_plant->GetFrameByName(final_frame_name).CalcPose(plant_context, multibody_plant->GetFrameByName("base_link")).GetAsMatrix4();
}

void AugmentedRigidArm::update_Jm(const VectorXd &q)
{
    double t0 = 0;
    double p0 = 0;
    double t1;
    double p1;
    for (int section_id = 0; section_id < st_params::num_segments * st_params::sections_per_segment; section_id ++)
    {
        /** @todo implement partial differentiation scheme */
        p1 = atan2(q(2 * section_id + 1), q(2 * section_id));
        t1 = sqrt(pow(q(section_id * 2), 2) + pow(q(section_id * 2 + 1), 2));
        int q_head = 2 * section_id;
        int xi_head = 5 * section_id;
        // Jm(xi_head, q_head) = ; // d(X axis)/d( )
    }
    
}

void AugmentedRigidArm::update_dJm(const VectorXd &q, const VectorXd &dq)
{
    //todo: verify this numerical method too
    //    double epsilon = 0.1;
    //    Vector2Nd q_delta = Vector2Nd(q);
    //    q_delta += dq * epsilon;
    //    update_Jm(q);
    //    Eigen::Matrix<double, JOINTS * N_SEGMENTS, 2 * N_SEGMENTS> Jxi_current = Eigen::Matrix<double,
    //            JOINTS * N_SEGMENTS, 2 * N_SEGMENTS>(Jm);
    //    update_Jm(q_delta);
    //    Eigen::Matrix<double, JOINTS * N_SEGMENTS, 2 * N_SEGMENTS> Jxi_delta = Eigen::Matrix<double,
    //            JOINTS * N_SEGMENTS, 2 * N_SEGMENTS>(Jm);
    //    dJm = (Jxi_delta - Jxi_current) / epsilon;
}

void AugmentedRigidArm::update(const VectorXd &q, const VectorXd &dq)
{
    assert(q.size() % st_params::num_segments == 0);
    assert(dq.size() % st_params::num_segments == 0);

    // first calculate m (augmented model parameters)
    calculate_m(q, xi);
    update_drake_model();
    update_Jm(q);
    //    update_dJm(q, dq);
    //
}

void AugmentedRigidArm::simulate()
{
    drake::systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
    simulator.set_publish_every_time_step(true);
    simulator.set_target_realtime_rate(1.0);
    simulator.Initialize();
    simulator.AdvanceTo(10);
}
