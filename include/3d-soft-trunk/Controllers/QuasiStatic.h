#pragma once

#include "3d-soft-trunk/ControllerPCC.h"

class QuasiStatic: public ControllerPCC{
public:
    QuasiStatic(const SoftTrunkParameters st_params);

private: 
    void control_loop();
    double kp;
    double kd;
    VectorXd p_prev = VectorXd::Zero(st_params_.p_pseudo_size);
    MatrixXd J;
    Vector3d ddx_des;
    VectorXd tau_ref;
};