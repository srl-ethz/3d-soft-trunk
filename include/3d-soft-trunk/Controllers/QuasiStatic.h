#pragma once

#include "3d-soft-trunk/ControllerPCC.h"

/** @brief Task Space Jacobian Controller using a Quasi-Static assumption */
class QuasiStatic: public ControllerPCC{
public:
    QuasiStatic(const SoftTrunkParameters st_params);
    double kp_;
    double kd_;

private: 
    void control_loop();
    VectorXd p_prev = VectorXd::Zero(st_params_.p_pseudo_size);
    MatrixXd J;
    Vector3d ddx_des;
    VectorXd tau_ref;
};