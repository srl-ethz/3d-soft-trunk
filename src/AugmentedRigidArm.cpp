// Copyright 2018 ...
#include "3d-soft-trunk/AugmentedRigidArm.h"

AugmentedRigidArm::AugmentedRigidArm() {
    setup_drake_model();
}

void AugmentedRigidArm::setup_drake_model() {
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
                                                                                               &scene_graph).AddModelFromFile(
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

    drake::geometry::DrakeVisualizer::AddToBuilder(&builder, scene_graph);

    diagram = builder.Build();
    diagram_context = diagram->CreateDefaultContext();

    // This is supposed to be required to visualize without simulation, but it does work without one...
    // drake::geometry::DrakeVisualizer::DispatchLoadMessage(scene_graph, lcm);

    int num_joints = multibody_plant->num_joints() - 1; // @todo: but why??
    fmt::print("model has {} joints\n", num_joints);
    joints_per_segment = num_joints / st_params::num_segments;

    // check that parameters make sense, just in case
    assert(st_params::num_segments == st_params::masses.size());
    assert(st_params::num_segments == st_params::lengths.size());
    assert(num_joints % st_params::num_segments == 0);
    if (st_params::rigidModel == RigidModelType::straw_bend)
        assert(joints_per_segment == 11);
    else
        assert(0); // not implemented yet

    // initialize variables
    xi = VectorXd::Zero(num_joints);
    B_xi = MatrixXd::Zero(num_joints, num_joints);
    G_xi = VectorXd::Zero(num_joints);
    Jm = MatrixXd::Zero(num_joints, 2 * st_params::num_segments);
    dJm = MatrixXd::Zero(num_joints, 2 * st_params::num_segments);
    Jxi = MatrixXd::Zero(3, num_joints);
    update_drake_model();
}

VectorXd AugmentedRigidArm::straw_bend_joint(double phi, double theta) {
    VectorXd angles = VectorXd::Zero(3);
    if (theta == 0)
        return angles;
    angles(0) = -atan(tan(theta) * sin(phi));
    angles(1) = asin(sin(theta) * cos(phi));
    angles(2) = -asin((cos(phi) * sin(phi) * cos(theta) - sin(phi) * cos(phi)) / cos(angles(1)));
    return angles;
}

void AugmentedRigidArm::calculate_m(const VectorXd &q, VectorXd &xi) {
    xi = VectorXd::Zero(joints_per_segment * st_params::num_segments);
    // intermediate variables useful for calculation
    double q_0;
    double q_1;
    int joint_id_head;
    double theta;
    double phi;
    double deltaL;
    for (int segment_id = 0; segment_id < st_params::num_segments; ++segment_id) {
        q_0 = q(2 * segment_id);
        q_1 = q(2 * segment_id + 1);
        if (st_params::parametrization == ParametrizationType::phi_theta) {
            phi = q_0;
            theta = q_1;
        } else if (st_params::parametrization == ParametrizationType::longitudinal) {
            // use phi, theta parametrization because it's simpler for calculation
            if (q_0 == 0) {
                if (q_1 == 0)
                    phi = 0;
                else
                    phi = PI / 2;
            }
            else
                phi = atan(q_1 / q_0);
            if (q_0 == 0)
                theta = -q_1 / cos(PI / 2 - phi);
            else
                theta = -q_0 / cos(phi);
            if (theta < 0){
                theta = -theta;
                phi += PI;
            }
        }

        joint_id_head = joints_per_segment * segment_id;
        if (st_params::rigidModel == RigidModelType::straw_bend) {
            double b = st_params::lengths[segment_id] / 2.;
            if (theta != 0)
                b = st_params::lengths[segment_id] / theta *
                    sqrt(1. + 4. * sin(theta / 2.) / theta * (sin(theta / 2.) / theta - cos(theta / 2.)));
            double nu = 0;
            if (theta != 0)
                nu = acos(1. / b * st_params::lengths[segment_id] / theta * sin(theta / 2.));
            deltaL = st_params::lengths[segment_id] / 2. - b;

            xi.segment(joint_id_head, 3) = straw_bend_joint(phi, theta / 2 - nu);
            xi(joints_per_segment * segment_id + 3) = deltaL;
            xi.segment(joint_id_head + 4, 3) = straw_bend_joint(phi, 2 * nu);
            xi(joints_per_segment * segment_id + 7) = deltaL;
            xi.segment(joint_id_head + 8, 3) = xi.segment(joint_id_head, 3);
        }
    }
}

void AugmentedRigidArm::update_drake_model() {
    // update drake model
    drake::systems::Context<double> &plant_context = diagram->GetMutableSubsystemContext(*multibody_plant,
                                                                                         diagram_context.get());
    multibody_plant->SetPositions(&plant_context, xi);

    // update drake visualization
    diagram->Publish(*diagram_context);

    // update some dynamic & kinematic params
    multibody_plant->CalcMassMatrix(plant_context, &B_xi);
    G_xi = multibody_plant->CalcGravityGeneralizedForces(plant_context);

    std::string final_frame_name = fmt::format("mid-{}", st_params::num_segments - 1);
    multibody_plant->CalcJacobianTranslationalVelocity(plant_context, drake::multibody::JacobianWrtVariable::kQDot,
                                                       multibody_plant->GetFrameByName(final_frame_name),
                                                       VectorXd::Zero(3), multibody_plant->world_frame(),
                                                       multibody_plant->world_frame(), &Jxi);
}

void AugmentedRigidArm::update_Jm(const VectorXd &q) {
    // brute force calculate Jacobian numerically
    // todo: verify that this numerical method is actually okay
    // this particular implementation only works because m of each element is totally independent of other elements
    VectorXd q_deltaX = q;
    VectorXd q_deltaY = q;
    VectorXd xi_current, xi_deltaX, xi_deltaY;
    double epsilon = 0.001;
    for (int i = 0; i < st_params::num_segments; ++i) {
        q_deltaX(2 * i) += epsilon;
        q_deltaY(2 * i + 1) += epsilon;
    }
    calculate_m(q, xi_current);
    calculate_m(q_deltaX, xi_deltaX);
    calculate_m(q_deltaY, xi_deltaY);
    for (int i = 0; i < st_params::num_segments; ++i) {
        Jm.block(joints_per_segment * i, 2 * i + 0, joints_per_segment, 1) = (xi_deltaX - xi_current).segment(
                joints_per_segment * i, joints_per_segment)/epsilon;
        Jm.block(joints_per_segment * i, 2 * i + 1, joints_per_segment, 1) = (xi_deltaY - xi_current).segment(
                joints_per_segment * i, joints_per_segment)/epsilon;
    }
}

void AugmentedRigidArm::update_dJm(const VectorXd &q, const VectorXd &dq) {
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

void AugmentedRigidArm::update(const VectorXd& q, const VectorXd& dq) {
    assert(q.size() % st_params::num_segments == 0);
    assert(dq.size() % st_params::num_segments == 0);

    // first calculate m (augmented model parameters)
    calculate_m(q, xi);
    update_drake_model();
    update_Jm(q);
//    update_dJm(q, dq);
//
}

void AugmentedRigidArm::simulate() {
    drake::systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
    simulator.set_publish_every_time_step(true);
    simulator.set_target_realtime_rate(1.0);
    simulator.Initialize();
    simulator.AdvanceTo(10);
}
