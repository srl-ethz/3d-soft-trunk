#pragma once

#include "ControllerPCC.h"
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
class IDCon: public ControllerPCC {
public:
    IDCon(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects = 0);
    /** @brief methods for getting OSC gain */
    double get_kp();
    double get_kd();

    /** @brief methods for setting OCS gain */
    void set_kp(double kp);
    void set_kd(double kd);
private:
    void control_loop();
    /** @brief gains for ID*/
    double kp;
    double kd;
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