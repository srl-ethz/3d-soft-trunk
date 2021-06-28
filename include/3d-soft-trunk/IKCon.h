#pragma once

#include "ControllerPCC.h"

class IKCon: public ControllerPCC {
public:
    IKCon(CurvatureCalculator::SensorType sensor_type, bool simulation = false, int objects = 0);
    /** @brief methods for getting OSC gain */
    double get_kp();
    double get_kd();

    /** @brief methods for setting OCS gain */
    void set_kp(double kp);
    void set_kd(double kd);
private:
    void control_loop();
    /** @brief gains for IK*/
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

};