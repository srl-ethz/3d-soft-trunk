#pragma once
#include <algorithm>
#include "mdefs.h"
#include "3d-soft-trunk/SoftTrunk_common.h"
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
    void Y_update(VectorXd q, VectorXd dq, VectorXd dqr, VectorXd ddqr);

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

    

public:
    Lagrange(const SoftTrunkParameters &st_params);

    /** @brief update the member variables based on current PCC value */
    void set_state(const srl::State &state);

    const SoftTrunkParameters st_params_;

    DynamicParams dyn_;
};
