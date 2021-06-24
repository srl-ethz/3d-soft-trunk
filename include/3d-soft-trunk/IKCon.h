#pragma once

#include "ControllerPCC.h"

class IKCon: public ControllerPCC {
public:
    IKCon(CurvatureCalculator::SensorType sensor_type, bool simulation = false, int objects = 0);

private:
    void control_loop();
    double kp;
    double kd;
    MatrixXd J;
    MatrixXd dJ;
    MatrixXd J_prev;
    Vector3d dx_prev;
    VectorXd ddx_ref;
    VectorXd tau_ref;
};