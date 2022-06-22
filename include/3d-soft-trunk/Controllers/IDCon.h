#pragma once

#include "3d-soft-trunk/ControllerPCC.h"
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>

/** @brief Inverse Dynamic Controller 
 * @details Similar to OSC, but uses Jacobian inversion instead of Operational Space Inertia Matrix */
class IDCon: public ControllerPCC {
public:
    IDCon(const SoftTrunkParameters st_params);
    double kp;
    double kd;

    void circle(int pressure, double period);

private:
    void control_loop();
    /** @brief gains for ID*/

    MatrixXd J;
    MatrixXd dJ;
    MatrixXd J_prev;
    MatrixXd J_inv;
    Vector3d dx_prev;
    Vector3d ddx_ref;
    Vector3d ddx_d;
    VectorXd tau_ref;
    double eps;
    double lambda;
    Eigen::MatrixXd computePinv(Eigen::MatrixXd j, double e, double lambda);
};