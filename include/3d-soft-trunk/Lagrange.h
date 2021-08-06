#pragma once
#include <algorithm>
#include "mdefs.h"
#include "SoftTrunk_common.h"
#include <iostream>

/**
 * @brief Represents the augmented rigid arm model.
 * @details The rigid arm model  approximates the soft arm. (see paper etc. for more info on how this is so)
 * This class can calculate the kinematics & dynamics of the augmented arm using Drake.
 * Values with an underscore at the end have extra 2 DoFs at the end of each segment, which represents the (always straight) connection pieces as another PCC section.
 * known issue: the values could get wrong when extremely close to straight configuration.
 */
class Lagrange {
private:
    const SoftTrunkParameters st_params;
    const double g0 = 9.80665;
    void A_update(VectorXd q);
    void M_update(VectorXd q);
    void g_update(VectorXd q);
    void c_update(VectorXd q, VectorXd dq);
    void k_update(VectorXd q);
    void d_update(VectorXd q, VectorXd dq);
    void p_update(VectorXd q);
    void J_update(VectorXd q);
    void JDot_update(VectorXd q, VectorXd dq);
    void Y_update(VectorXd q, VectorXd dq, VectorXd ddq);

public:
    Lagrange(const SoftTrunkParameters &st_params);

    /** @brief update the member variables based on current PCC value */
    void update(const srl::State &state, const srl::State &state_r);

    /** @brief torque mapping matrix */
    MatrixXd A;
    /** @brief inertia matrix */
    MatrixXd M;
    /** @brief coriolis and centrifugal term */
    VectorXd Cdq;
    /** @brief gravity term */
    VectorXd g;
    /** @brief damping term */
    VectorXd d;
    /** @brief stiffness term */
    VectorXd k;
    /** @brief tip position */
    VectorXd p;
    /** @brief tip Jacobian */
    MatrixXd J;
    /** @brief tip JacobianDot */
    MatrixXd JDot;   
    /** @brief Regressor Matrix */
    MatrixXd Y;
};
