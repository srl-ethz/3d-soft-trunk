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
 * This class can calculate the kinematics & dynamics of the augmented arm using Drake.
 * Values with an underscore at the end have extra 2 DoFs at the end of each segment, which represents the (always straight) connection pieces as another PCC section.
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

    /** @brief calculate joint angles of rigid model, for a PCC configuration q_. */
    void calculate_m(VectorXd q_);

    /** @brief update the Drake model using the current xi_, and calculate dynamic parameters B_xi_ and G_xi_. */
    void update_drake_model();

    void update_Jm(VectorXd q_);

    void update_dJm(const VectorXd &q_, const VectorXd &dq_);

    // these internally used values have extra PCC section at end of each segment, whose values are always set to 0.

    /** @brief current joint angles of the augmented rigid arm */
    MatrixXd xi_;
    MatrixXd dxi_;
    /** @brief the Jacobian that maps from q_ to xi_ */
    MatrixXd Jm_;
    /** @brief time derivative of Jm_ */
    MatrixXd dJm_;
    /** @brief Jacobian of tip position vs xi_ */
    MatrixXd Jxi_;
    /** @brief inertia matrix */
    MatrixXd B_xi_;
    /** @brief Coriolis, centripital & gyroscopic effects */
    VectorXd c_xi_;
    /** @brief gravity */
    MatrixXd g_xi_;
    /**
     * @brief matrix that maps from the [2 Nsegments] DoF parameters to [2 (Nsegment+1)] DoF parameters used internally.
     * e.g. q_ = map_normal2expanded * q
     */
    MatrixXd map_normal2expanded;

    /** @brief holds relative transforms between tip of each segment (tip of curving section, disregards the connector part) and base */
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> H_list;

public:
    AugmentedRigidArm();

    /** @brief update the member variables based on current PCC value */
    void update(const VectorXd &q, const VectorXd &dq);

    /** @brief simulate the rigid body model in Drake. The prismatic joints are broken... */
    void simulate();

    /** @brief inertia matrix mapped to q */
    MatrixXd B;
    /** @brief coreolis & centrifugal forces mapped to q */
    VectorXd c;
    /** @brief gravity mapped to q */
    MatrixXd g;
    /** @brief Jacobian of tip position vs q */
    MatrixXd J;

    /** @brief the pose of the rigid body at the tip of segment i, of the Drake model (relative to base frame), i.e. tip position for segment i calculated with forward kinematics using the PCC approximation. 
     * @param segment segment id. 0 for first segment, and so on
    */
    Eigen::Transform<double, 3, Eigen::Affine> get_H(int segment);

    /** @brief Convenience function that calls get_H for the tip segment. */
    Eigen::Transform<double, 3, Eigen::Affine> get_H_tip();
};
