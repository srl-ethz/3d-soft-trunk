#pragma once

#include <drake/common/drake_assert.h>
#include <drake/common/find_resource.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

#include "mdefs.h"
#include "SoftTrunk_common.h"

/**
 * @brief Represents the augmented rigid arm model.
 * @details The rigid arm model  approximates the soft arm. (see paper etc. for more info on how this is so)
 * This class can calculate the kinematics & dynamics of the augmented arm using RBDL.
 */
class AugmentedRigidArm {
private:
    // drake variables start
    /** @brief this builder helps in adding & connecting system blocks */
    drake::systems::DiagramBuilder<double> builder;
    /** @brief SceneGraph manages all the systems that uses geometry
     * https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_scene_graph.html
     * */
    drake::geometry::SceneGraph<double> &scene_graph = *builder.AddSystem<drake::geometry::SceneGraph>();
    /** @brief MultibodyPlant is the model of interconnected bodies.
     * https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html
     */
    drake::multibody::MultibodyPlant<double> *multibody_plant = builder.AddSystem<drake::multibody::MultibodyPlant<double>>(
            1.0e-3);
    std::unique_ptr<drake::systems::Diagram<double>> diagram;
    std::unique_ptr<drake::systems::Context<double>> diagram_context;
    // drake variables end

    /**
     * @brief load from URDF and set up Drake model
     */
    void setup_drake_model();

    int joints_per_segment;

    /**
     * @brief convert phi-theta bend to joint angles for a straw-bend joint.
     */
    VectorXd straw_bend_joint(double phi, double theta);

    /** @brief calculate joint angles of rigid model, for a PCC configuration q. */
    void calculate_m(const VectorXd &q, VectorXd &xi);

    /** @brief update the Drake model using the current xi, and calculate dynamic parameters B_xi and G_xi. */
    void update_drake_model();

    void update_Jm(const VectorXd &q);

    void update_dJm(const VectorXd &q, const VectorXd &dq);

    void update_Jxi(const VectorXd &q);

public:
    AugmentedRigidArm();

    /** @brief update the member variables based on current PCC value */
    void update(const VectorXd &q, const VectorXd &dq);

    /** @brief simulate the rigid body model in Drake. The prismatic joints are broken... */
    void simulate();

    /** @brief current joint angles of the augmented rigid arm */
    VectorXd xi;

    /** @brief the Jacobian that maps from q to xi */
    MatrixXd Jm;

    /** @brief the time derivative of the Jacobian that maps from q to xi */
    MatrixXd dJm;

    MatrixXd Jxi;

    /** @brief inertia matrix */
    MatrixXd B_xi;

    /** @brief the gravity vector, i.e. the force at each joint when the arm is completely stationary at its current configuration. */
    MatrixXd G_xi;

    /** @brief the pose of the rigid body at the tip of the Drake model (relative to base frame), i.e. tip position calculated with forward kinematics using the PCC approximation. */
    Eigen::Transform<double, 3, Eigen::Affine> H_tip;
};
