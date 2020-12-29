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
     * @param q_ref
     * @param dq_ref
     * @param ddq_ref
     */
    void set_ref(
            const VectorXd &q_ref,
            const VectorXd &dq_ref,
            const VectorXd &ddq_ref);

    void get_status(VectorXd &q, VectorXd dq, VectorXd &p_vectorized);
    void get_jacobian(MatrixXd &J);

private:
    std::unique_ptr<AugmentedRigidArm> ara;
    std::unique_ptr<ValveController> vc;
    std::unique_ptr<CurvatureCalculator> cc;

    VectorXd K; /** @brief stiffness coefficient of silicone arm */
    VectorXd D; /** @brief damping coefficient of silicone arm */
    VectorXd K_p; /** @brief P gain for pose FB */
    VectorXd K_d; /** @brief D gain for pose FB */
    VectorXd alpha; /** @brief used to convert torque to pressure (pressure = torque / alpha). Not used for PID control */

    std::vector<MiniPID> miniPIDs;
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

    VectorXd q;
    VectorXd dq;
    VectorXd ddq;
    VectorXd q_ref;
    VectorXd dq_ref;
    VectorXd ddq_ref;

    VectorXd p_vectorized; /** @brief vector that expresses net pressure for X&Y directions, for each segment */

    /**
     * @brief update B(inertia matrix), C and G(gravity vector) in q space
     */
    void updateBCG(const VectorXd &q, const VectorXd &dq);

    MatrixXd B;
    MatrixXd C;
    VectorXd G;
    MatrixXd J; // Eigen::Matrix<double, 3, 2*N_SEGMENTS>
};