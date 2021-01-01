//
// Created by yasu on 26/10/18.
//

#pragma once

#include "SoftTrunk_common.h"
#include <AugmentedRigidArm.h>
#include "mobilerack-interface/ValveController.h"
#include "CurvatureCalculator.h"
#include "MiniPID.h"
#include <mutex>

/**
 * @brief Implements the PCC controller as described in paper.
 * @details It receives pointers to instances of AugmentedRigidArm and SoftArm, so it can access instances of those classes to retrieve information about them that can be used in the Manager class to control the Soft Trunk.
 * By setting USE_PID_CURVATURE_CONTROL to true in SoftTrunk_common_defs.h, it can also do PID control.
 * @todo it is in the midst of conversion process to new system, doesn't work for now.
 */
class ControllerPCC {
public:
    ControllerPCC();

    /** @brief set the reference pose (trajectory) of the arm
     */
    void set_ref(
            const VectorXd &q_ref,
            const VectorXd &dq_ref,
            const VectorXd &ddq_ref);

    /** @brief get current kinematic parameters of the arm (pose and Jacobian) */
    void get_kinematic(VectorXd &q, VectorXd dq, MatrixXd &J);
    /** @brief get current dynamic parameters of the arm
     * @param B inertia matrix
     * @param C
     * @param G gravity torque
     */
    void get_dynamic(MatrixXd &B, MatrixXd &C, MatrixXd &G);
    /** @brief get current pressure output to the arm */
    void get_pressure(VectorXd &p_vectorized);

private:
    std::unique_ptr<AugmentedRigidArm> ara;
    std::unique_ptr<ValveController> vc;
    std::unique_ptr<CurvatureCalculator> cc;

    // parameters for dynamic controller
    VectorXd K; /** @brief stiffness coefficient of silicone arm */
    VectorXd D; /** @brief damping coefficient of silicone arm */
    VectorXd K_p; /** @brief P gain for pose FB */
    VectorXd K_d; /** @brief D gain for pose FB */
    double alpha = 2.16e-4; /** @brief used to convert generalized force tau to pressure P (P = tau / alpha). Not used for PID control. Uses model-based value, which is very close to experimental value 2.88e-4. */
    double beta; // ????

    // parameters for PID controller
    std::array<double, 3> Ku = {800, 500, 400}; /** @brief ultimate gain for each segment, used in Ziegler-Nichols method */
    std::array<double, 3> Tu = {0.7, 0.7, 0.7}; /** @brief oscillation period for each segment, used in Ziegler-Nichols method */
    std::vector<MiniPID> miniPIDs;

    /** @brief baseline pressure of arm. The average of the pressures sent to a segment should be this pressure.
     * for DragonSkin 30, set to 300.
     * for DragonSkin 10, set to 150.
     * (not throughly examined- a larger or smaller value may be better)
     */
    const int p_offset = 150;
    const int p_max = 400; // 400 for DS 10, 1200 for DS 30

    bool use_feedforward = false;
    bool simulate = false;

    const double dt = 1./30.;

    bool is_initial_ref_received = false;

    /**
     * actuate the arm using generalized forces
     * @param f generalized force expressed in st_params::parametrization space
     */
    void actuate(VectorXd f);

    std::thread control_thread;
    std::mutex mtx;

    void control_loop();

    // arm configuration
    VectorXd q;
    VectorXd dq;
    VectorXd ddq;
    VectorXd q_ref;
    VectorXd dq_ref;
    VectorXd ddq_ref;

    VectorXd p_vectorized; /** @brief vector that expresses net pressure for X&Y directions, for each segment */

    /**
     * @brief calculate dynamic & kinematic parameters using augmented rigid model
     */
    void updateBCGJ(const VectorXd &q, const VectorXd &dq);

    MatrixXd B; /** @brief inertia matrix */
    MatrixXd C;
    VectorXd G; /** @brief gravity vector */
    MatrixXd J; /** @brief Jacobian for tip of arm */
};