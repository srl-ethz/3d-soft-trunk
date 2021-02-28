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
    world_to_base.set_rotation(drake::math::RollPitchYaw(0., st_params::armAngle*PI/180., 0.));
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

    int num_joints = multibody_plant->num_joints() - 2; // subtract one mystery joint, and one fixed joint at the base
    fmt::print("model has {} joints\n", num_joints);

    // check that parameters make sense, just in case
    assert(st_params::num_segments * 2 == st_params::lengths.size());
    assert(st_params::num_segments + 1 == st_params::diameters.size());
    assert(num_joints == 5 * st_params::num_segments * (st_params::sections_per_segment + 1));

    // initialize variables
    xi_ = VectorXd::Zero(num_joints);
    dxi_ = VectorXd::Zero(num_joints);
    B_xi_ = MatrixXd::Zero(num_joints, num_joints);
    c_xi_ = VectorXd::Zero(num_joints);
    g_xi_ = VectorXd::Zero(num_joints);
    Jm_ = MatrixXd::Zero(num_joints, 2 * st_params::num_segments * (st_params::sections_per_segment+1));
    dJm_ = MatrixXd::Zero(num_joints, 2 * st_params::num_segments * (st_params::sections_per_segment+1));
    Jxi_ = MatrixXd::Zero(3, num_joints);
    H_list.resize(st_params::num_segments);

    map_normal2expanded = MatrixXd::Zero(2*st_params::num_segments*(st_params::sections_per_segment + 1), 2*st_params::num_segments*st_params::sections_per_segment);
    for (int i = 0; i < st_params::num_segments; i++)
      map_normal2expanded.block(2*i*(st_params::sections_per_segment + 1), 2*i*st_params::sections_per_segment, 2*st_params::sections_per_segment, 2*st_params::sections_per_segment) = MatrixXd::Identity(2*st_params::sections_per_segment, 2*st_params::sections_per_segment);
    update_drake_model();
}

void AugmentedRigidArm::calculate_m(VectorXd q_)
{
    // intermediate variables useful for calculation
    // use phi, theta parametrization because it's simpler for calculation
    double p0 = 0; // phi of previous section
    double t0 = 0; // theta of previous section
    double p1; // phi of current section
    double t1; // theta of current section
    double l; // length of current section
    int joint_id_head; // index of first joint in section (5 joints per section)
    int segment_id;
    for (int section_id = 0; section_id < st_params::num_segments * (st_params::sections_per_segment + 1); ++section_id)
    {
        segment_id = section_id / (st_params::sections_per_segment + 1);
        longitudinal2phiTheta(q_(2*section_id), q_(2*section_id + 1), p1, t1);
        t1 = std::max(0.0001, t1); /** @todo hack way to get rid of errors when close to straight. */
        joint_id_head = 5 * section_id;
        l = st_params::lengths[2 * segment_id] / st_params::sections_per_segment;
        // calculate joint angles that kinematically and dynamically match
        // this was calculated from Mathematica and not by hand
        xi_(joint_id_head) = -ArcSin((Cos(t1/2.)*Sin(p0)*Sin(t0/2.) - Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) + Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.))/
     Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2)));
        
        xi_(joint_id_head + 1) = ArcSin(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.));
        
        xi_(joint_id_head + 2) = -ArcSin((-(Cos(p0)*Power(Cos(p1),2)*Sin(p0)) + Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Sin(p0) - Cos(p0)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Power(Cos(p1),2)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - 
       Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - Cos(p1)*Cos(t0/2.)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p1) - Cos(p1)*Power(Sin(p0),2)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1) + 
       Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.))/
     Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2)));

        xi_(joint_id_head + 3) = l / 2 - l * sin((t1) / 2) / t1;
        xi_(joint_id_head + 4) = xi_(joint_id_head + 3);
        t0 = t1;
        p0 = p1;
    }
}

void AugmentedRigidArm::update_drake_model()
{
    // update drake model
    drake::systems::Context<double> &plant_context = diagram->GetMutableSubsystemContext(*multibody_plant,
                                                                                         diagram_context.get());
    multibody_plant->SetPositions(&plant_context, xi_);
    multibody_plant->SetVelocities(&plant_context, dxi_);

    // update drake visualization
    diagram->Publish(*diagram_context);

    // update some dynamic & kinematic params
    multibody_plant->CalcMassMatrix(plant_context, &B_xi_);
    multibody_plant->CalcBiasTerm(plant_context, &c_xi_);
    g_xi_ = - multibody_plant->CalcGravityGeneralizedForces(plant_context);

    std::string frame_name;
    // for end of each segment, calculate the FK position
    for (int i = 0; i < st_params::num_segments; i++)
    {
      frame_name = fmt::format("seg{}_sec{}-{}_connect", i, st_params::sections_per_segment-1, st_params::sections_per_segment);
      H_list[i] = multibody_plant->GetFrameByName(frame_name).CalcPose(plant_context, multibody_plant->GetFrameByName("base_link")).GetAsMatrix4();
      if (i == st_params::num_segments - 1){
        // for the final segment, calculate the Jacobian as well
        multibody_plant->CalcJacobianTranslationalVelocity(plant_context, drake::multibody::JacobianWrtVariable::kQDot,
                                                       multibody_plant->GetFrameByName(frame_name),
                                                       VectorXd::Zero(3), multibody_plant->world_frame(),
                                                       multibody_plant->world_frame(), &Jxi_);
      }
    }
}

/**
 * @brief calculate partial differentiation of phi & theta w.r.t. Lx, Ly.
 * [[d(phi)/d(Lx), d(phi)/d(Ly)],
 *  [d(theta)/d(Lx), d(theta)/d(Ly)]]
 * @param Lx Lx of longitudinal parametrization
 * @param Ly Lx of longitudinal parametrization
 * @param M result. Must be 2x2 matrix
 */
void calcPhiThetaDiff(double Lx, double Ly, MatrixXd& M){
    if (-0.0001 < Lx && Lx < 0.0001 && -0.0001 < Ly && Ly < 0.0001)
      Lx = 0.0001; // hack way to get rid of errors when Lx & Ly are small
    assert(M.rows() == 2 && M.cols() == 2);
    double tmp =  (pow(Lx,2) + pow(Ly,2));
    M(0,0) = - Ly/tmp;
    M(0,1) = Lx / tmp; 
    M(1,0) = Lx / sqrt(tmp);
    M(1,1) = Ly / sqrt(tmp);
}

void AugmentedRigidArm::update_Jm(VectorXd q_)
{
    double p0 = 0;
    double t0 = 0;
    double p1;
    double t1;
    double l;
    int segment_id;
    MatrixXd dxi_dpt = MatrixXd::Zero(5, 2); // d(xi)/d(phi, theta)
    MatrixXd dpt_dL = MatrixXd::Zero(2, 2); // d(phi, theta)/d(Lx, Ly)
    for (int section_id = 0; section_id < st_params::num_segments * (st_params::sections_per_segment + 1); section_id ++)
    {
        segment_id = section_id / (st_params::sections_per_segment+1);
        // differentiation is calculated via phi-theta parametrization for easier formulation.
        int q_head = 2 * section_id;
        int xi_head = 5 * section_id;
        l = st_params::lengths[2 * segment_id] / st_params::sections_per_segment;
        longitudinal2phiTheta(q_(q_head), q_(q_head+1), p1, t1);
        /** @todo this is a hack way to get rid of computation errors when values are 0 */
        t1 = std::max(0.0001, t1);
        if (section_id != 0)
        {
            // Calculated from Mathematica
            // d(rot_x)/d(phi0)
            dxi_dpt(0,0) = -((((Cos(t1/2.)*Sin(p0)*Sin(t0/2.) - Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) + Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.))*
          (Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.))*
          (-(Cos(t1/2.)*Sin(p0)*Sin(t0/2.)) + 2*Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) - 2*Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) - Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.) + 
            Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.) - Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.)))/
        Power(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2),1.5) + (Cos(p0)*Cos(t1/2.)*Sin(t0/2.) - Power(Cos(p0),2)*Cos(p1)*Sin(t1/2.) + Power(Cos(p0),2)*Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - 
          2*Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + 2*Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.))/
        Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2)))/Sqrt(1 - Power(Cos(t1/2.)*Sin(p0)*Sin(t0/2.) - Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) + Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - 
          Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.),2)/
        (1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2))));
            
            // d(rot_x)/d(theta0)
            dxi_dpt(0,1) = -((((Cos(t1/2.)*Sin(p0)*Sin(t0/2.) - Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) + Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.))*
          (Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.))*
          ((Cos(p0)*Cos(t0/2.)*Cos(t1/2.))/2. - (Cos(p1)*Sin(t0/2.)*Sin(t1/2.))/2. + (Cos(p1)*Power(Sin(p0),2)*Sin(t0/2.)*Sin(t1/2.))/2. - (Cos(p0)*Sin(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.))/2.))/
        Power(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2),1.5) + ((Cos(t0/2.)*Cos(t1/2.)*Sin(p0))/2. - (Cos(p0)*Cos(p1)*Sin(p0)*Sin(t0/2.)*Sin(t1/2.))/2. - (Sin(p1)*Sin(t0/2.)*Sin(t1/2.))/2. + (Power(Cos(p0),2)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.))/2.)/
        Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2)))/Sqrt(1 - Power(Cos(t1/2.)*Sin(p0)*Sin(t0/2.) - Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) + Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - 
          Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.),2)/
        (1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2))));

            // d(rot_y)/d(phi0)
            dxi_dpt(1,0) = (-(Cos(t1/2.)*Sin(p0)*Sin(t0/2.)) + 2*Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) - 2*Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) - Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.) + Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.) - 
     Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.))/
   Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2));

            // d(rot_y)/d(theta0)
            dxi_dpt(1,1) = ((Cos(p0)*Cos(t0/2.)*Cos(t1/2.))/2. - (Cos(p1)*Sin(t0/2.)*Sin(t1/2.))/2. + (Cos(p1)*Power(Sin(p0),2)*Sin(t0/2.)*Sin(t1/2.))/2. - (Cos(p0)*Sin(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.))/2.)/
   Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2));

            // d(rot_z)/d(phi0)
            dxi_dpt(2,0) = ((((Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.))*
          (-(Cos(t1/2.)*Sin(p0)*Sin(t0/2.)) + 2*Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) - 2*Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) - Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.) + 
            Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.) - Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.))*(-(Cos(p0)*Power(Cos(p1),2)*Sin(p0)) + Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Sin(p0) - Cos(p0)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Power(Cos(p1),2)*Cos(t1/2.)*Sin(p0) + 
            Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - Cos(p1)*Cos(t0/2.)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p1) - Cos(p1)*Power(Sin(p0),2)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1) + 
            Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.)))/
        Power(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2),1.5) + (-(Power(Cos(p0),2)*Power(Cos(p1),2)) + Power(Cos(p0),2)*Power(Cos(p1),2)*Cos(t0/2.) - Power(Cos(p0),2)*Cos(t1/2.) + Power(Cos(p0),2)*Power(Cos(p1),2)*Cos(t1/2.) + Power(Cos(p0),2)*Cos(t0/2.)*Cos(t1/2.) - 
          Power(Cos(p0),2)*Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.) + Power(Cos(p1),2)*Power(Sin(p0),2) - Power(Cos(p1),2)*Cos(t0/2.)*Power(Sin(p0),2) + Cos(t1/2.)*Power(Sin(p0),2) - Power(Cos(p1),2)*Cos(t1/2.)*Power(Sin(p0),2) - 
          Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2) + Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2) - 2*Cos(p0)*Cos(p1)*Sin(p0)*Sin(p1) + 2*Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(p1) + 2*Cos(p0)*Cos(p1)*Cos(t1/2.)*Sin(p0)*Sin(p1) - 
          2*Cos(p0)*Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0)*Sin(p1) + Sin(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.))/
        Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2)))/Sqrt(1 - Power(-(Cos(p0)*Power(Cos(p1),2)*Sin(p0)) + Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Sin(p0) - Cos(p0)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Power(Cos(p1),2)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - 
          Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - Cos(p1)*Cos(t0/2.)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p1) - Cos(p1)*Power(Sin(p0),2)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1) + 
          Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.),2)/
        (1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2))));

            // d(rot_z)/d(theta0)
            dxi_dpt(2,1) = -((((Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.))*
          (-(Cos(p0)*Power(Cos(p1),2)*Sin(p0)) + Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Sin(p0) - Cos(p0)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Power(Cos(p1),2)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - 
            Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - Cos(p1)*Cos(t0/2.)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p1) - Cos(p1)*Power(Sin(p0),2)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1) + 
            Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.))*
          ((Cos(p0)*Cos(t0/2.)*Cos(t1/2.))/2. - (Cos(p1)*Sin(t0/2.)*Sin(t1/2.))/2. + (Cos(p1)*Power(Sin(p0),2)*Sin(t0/2.)*Sin(t1/2.))/2. - (Cos(p0)*Sin(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.))/2.))/
        Power(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2),1.5) + (-0.5*(Cos(p0)*Power(Cos(p1),2)*Sin(p0)*Sin(t0/2.)) - (Cos(p0)*Cos(t1/2.)*Sin(p0)*Sin(t0/2.))/2. + (Cos(p0)*Power(Cos(p1),2)*Cos(t1/2.)*Sin(p0)*Sin(t0/2.))/2. + (Cos(p1)*Sin(p1)*Sin(t0/2.))/2. - 
          (Cos(p1)*Cos(t1/2.)*Sin(p1)*Sin(t0/2.))/2. - (Cos(p1)*Power(Sin(p0),2)*Sin(p1)*Sin(t0/2.))/2. + (Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1)*Sin(t0/2.))/2. - (Cos(p0)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.))/2.)/
        Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2)))/Sqrt(1 - Power(-(Cos(p0)*Power(Cos(p1),2)*Sin(p0)) + Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Sin(p0) - Cos(p0)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Power(Cos(p1),2)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - 
          Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - Cos(p1)*Cos(t0/2.)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p1) - Cos(p1)*Power(Sin(p0),2)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1) + 
          Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.),2)/
        (1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2))));

            // d(prismatic)/d(phi0)
            dxi_dpt(3,0) = 0;
            dxi_dpt(4,0) = 0;

            // d(prismatic)/d(theta0)
            dxi_dpt(3,1) = 0;
            dxi_dpt(4,1) = 0;

            calcPhiThetaDiff(q_(q_head - 2), q_(q_head-1), dpt_dL);
            Jm_.block(xi_head, q_head-2, 5, 2) = dxi_dpt * dpt_dL;
        }
        // Calculated from Mathematica
        // d(rot_x)/d(phi1)
        dxi_dpt(0,0) = -((((Cos(t1/2.)*Sin(p0)*Sin(t0/2.) - Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) + Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.))*
          (Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.))*
          (-(Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.)) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) - Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.)))/
        Power(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2),1.5) + (Power(Cos(p0),2)*Cos(p1)*Sin(t1/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) - Power(Cos(p0),2)*Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) - Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.))/
        Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2)))/Sqrt(1 - Power(Cos(t1/2.)*Sin(p0)*Sin(t0/2.) - Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) + Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - 
          Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.),2)/
        (1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2))));
        
        // d(rot_x)/d(theta1)
        dxi_dpt(0,1) = -((((Cos(t1/2.)*Sin(p0)*Sin(t0/2.) - Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) + Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.))*
          (Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.))*
          ((Cos(p1)*Cos(t0/2.)*Cos(t1/2.))/2. + (Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2))/2. - (Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2))/2. - (Cos(p0)*Cos(t1/2.)*Sin(p0)*Sin(p1))/2. + (Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0)*Sin(p1))/2. - 
            (Cos(p0)*Sin(t0/2.)*Sin(t1/2.))/2.))/Power(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + 
            Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2),1.5) + (-0.5*(Cos(p0)*Cos(p1)*Cos(t1/2.)*Sin(p0)) + (Cos(p0)*Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0))/2. + (Power(Cos(p0),2)*Cos(t1/2.)*Sin(p1))/2. + (Cos(t0/2.)*Cos(t1/2.)*Sin(p1))/2. - 
          (Power(Cos(p0),2)*Cos(t0/2.)*Cos(t1/2.)*Sin(p1))/2. - (Sin(p0)*Sin(t0/2.)*Sin(t1/2.))/2.)/
        Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2)))/Sqrt(1 - Power(Cos(t1/2.)*Sin(p0)*Sin(t0/2.) - Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) + Power(Cos(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - 
          Power(Cos(p0),2)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.),2)/
        (1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2))));
        
        // d(rot_y)/d(phi1)
        dxi_dpt(1,0) = (-(Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.)) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) - Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.))/
   Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2));

        // d(rot_y)/d(theta1)
        dxi_dpt(1,1) = ((Cos(p1)*Cos(t0/2.)*Cos(t1/2.))/2. + (Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2))/2. - (Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2))/2. - (Cos(p0)*Cos(t1/2.)*Sin(p0)*Sin(p1))/2. + (Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0)*Sin(p1))/2. - 
     (Cos(p0)*Sin(t0/2.)*Sin(t1/2.))/2.)/Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + 
       Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2));

        // d(rot_z)/d(phi1)
        dxi_dpt(2,0) = -((((Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.))*
          (-(Cos(p0)*Cos(p1)*Sin(p0)*Sin(t1/2.)) + Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.) - Cos(t0/2.)*Sin(p1)*Sin(t1/2.) - Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.) + Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.))*
          (-(Cos(p0)*Power(Cos(p1),2)*Sin(p0)) + Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Sin(p0) - Cos(p0)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Power(Cos(p1),2)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - 
            Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - Cos(p1)*Cos(t0/2.)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p1) - Cos(p1)*Power(Sin(p0),2)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1) + 
            Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.)))/
        Power(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2),1.5) + (-(Power(Cos(p1),2)*Cos(t0/2.)) + Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.) - Power(Cos(p1),2)*Power(Sin(p0),2) + Power(Cos(p1),2)*Cos(t0/2.)*Power(Sin(p0),2) + Power(Cos(p1),2)*Cos(t1/2.)*Power(Sin(p0),2) - 
          Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2) + 2*Cos(p0)*Cos(p1)*Sin(p0)*Sin(p1) - 2*Cos(p0)*Cos(p1)*Cos(t0/2.)*Sin(p0)*Sin(p1) - 2*Cos(p0)*Cos(p1)*Cos(t1/2.)*Sin(p0)*Sin(p1) + 2*Cos(p0)*Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0)*Sin(p1) + 
          Cos(t0/2.)*Power(Sin(p1),2) - Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p1),2) + Power(Sin(p0),2)*Power(Sin(p1),2) - Cos(t0/2.)*Power(Sin(p0),2)*Power(Sin(p1),2) - Cos(t1/2.)*Power(Sin(p0),2)*Power(Sin(p1),2) + 
          Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2)*Power(Sin(p1),2) - Cos(p0)*Cos(p1)*Sin(t0/2.)*Sin(t1/2.))/
        Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2)))/Sqrt(1 - Power(-(Cos(p0)*Power(Cos(p1),2)*Sin(p0)) + Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Sin(p0) - Cos(p0)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Power(Cos(p1),2)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - 
          Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - Cos(p1)*Cos(t0/2.)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p1) - Cos(p1)*Power(Sin(p0),2)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1) + 
          Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.),2)/
        (1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2))));

        // d(rot_z)/d(theta1)
        dxi_dpt(2,1) = -((((Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.))*
          ((Cos(p1)*Cos(t0/2.)*Cos(t1/2.))/2. + (Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2))/2. - (Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2))/2. - (Cos(p0)*Cos(t1/2.)*Sin(p0)*Sin(p1))/2. + (Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0)*Sin(p1))/2. - 
            (Cos(p0)*Sin(t0/2.)*Sin(t1/2.))/2.)*(-(Cos(p0)*Power(Cos(p1),2)*Sin(p0)) + Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Sin(p0) - Cos(p0)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Power(Cos(p1),2)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - 
            Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - Cos(p1)*Cos(t0/2.)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p1) - Cos(p1)*Power(Sin(p0),2)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1) + 
            Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.)))/
        Power(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2),1.5) + (-0.5*(Cos(p0)*Cos(t1/2.)*Sin(p1)*Sin(t0/2.)) + (Cos(p0)*Sin(p0)*Sin(t1/2.))/2. - (Cos(p0)*Power(Cos(p1),2)*Sin(p0)*Sin(t1/2.))/2. - (Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.))/2. + 
          (Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Sin(p0)*Sin(t1/2.))/2. - (Cos(p1)*Cos(t0/2.)*Sin(p1)*Sin(t1/2.))/2. - (Cos(p1)*Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.))/2. + (Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1)*Sin(t1/2.))/2.)/
        Sqrt(1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),
           2)))/Sqrt(1 - Power(-(Cos(p0)*Power(Cos(p1),2)*Sin(p0)) + Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Sin(p0) - Cos(p0)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Power(Cos(p1),2)*Cos(t1/2.)*Sin(p0) + Cos(p0)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - 
          Cos(p0)*Power(Cos(p1),2)*Cos(t0/2.)*Cos(t1/2.)*Sin(p0) - Cos(p1)*Cos(t0/2.)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Sin(p1) - Cos(p1)*Power(Sin(p0),2)*Sin(p1) + Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(p1) + 
          Cos(p1)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p1)*Cos(t0/2.)*Cos(t1/2.)*Power(Sin(p0),2)*Sin(p1) - Cos(p0)*Sin(p1)*Sin(t0/2.)*Sin(t1/2.),2)/
        (1 - Power(Cos(p0)*Cos(t1/2.)*Sin(t0/2.) + Cos(p1)*Cos(t0/2.)*Sin(t1/2.) + Cos(p1)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p1)*Cos(t0/2.)*Power(Sin(p0),2)*Sin(t1/2.) - Cos(p0)*Sin(p0)*Sin(p1)*Sin(t1/2.) + Cos(p0)*Cos(t0/2.)*Sin(p0)*Sin(p1)*Sin(t1/2.),2))));
        // d(prismatic)/d(phi1)
        dxi_dpt(3,0) = 0;
        dxi_dpt(4,0) = 0;
        // d(prismatic)/d(theta1)
        dxi_dpt(3,1) = -0.5*(l*Cos(t1/2.))/t1 + (l*Sin(t1/2.))/Power(t1,2);
        dxi_dpt(4,1) = dxi_dpt(3,1);
        
        calcPhiThetaDiff(q_(q_head), q_(q_head+1), dpt_dL);
        Jm_.block(xi_head, q_head, 5, 2) = dxi_dpt * dpt_dL;
        p0 = p1;
        t0 = t1;
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
    assert(q.size() == 2 * st_params::num_segments * st_params::sections_per_segment);
    assert(dq.size() == q.size());

    // calculate rigid model pose
    calculate_m(map_normal2expanded * q);
    // calculate Jacobian
    update_Jm(map_normal2expanded * q);
    dxi_ = Jm_ * map_normal2expanded * dq;
    // calculate dynamic parameters
    update_drake_model();
    // map to q space
    B = map_normal2expanded.transpose() * (Jm_.transpose() * B_xi_ * Jm_) * map_normal2expanded;
    c = map_normal2expanded.transpose() * (Jm_.transpose() * c_xi_);
    g = map_normal2expanded.transpose() * (Jm_.transpose() * g_xi_);
    J = Jxi_ * Jm_ * map_normal2expanded;
    //    update_dJm(q, dq);
    //
}

Eigen::Transform<double, 3, Eigen::Affine> AugmentedRigidArm::get_H(int segment){
    assert(0 <= segment && segment < st_params::num_segments);
    return H_list[segment];
}

Eigen::Transform<double, 3, Eigen::Affine> AugmentedRigidArm::get_H_tip(){
    return get_H(st_params::num_segments - 1);
}


void AugmentedRigidArm::simulate()
{
    drake::systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
    simulator.set_publish_every_time_step(true);
    simulator.set_target_realtime_rate(1.0);
    simulator.Initialize();
    simulator.AdvanceTo(10);
}
