#pragma once

#include "3d-soft-trunk/ControllerPCC.h"

class QuasiStatic: public ControllerPCC{
public:
    QuasiStatic(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects = 0);


    /** @brief get kp gain */    
    double get_kp();
    /** @brief get kd gain */
    double get_kd();

    /** @brief set kp gain */
    void set_kp(double kp);
    /** @brief set kd gain */
    void set_kd(double kd);

protected:
    int singularity(const MatrixXd &J);

private: 
    void control_loop();
    double kp;
    double kd;
    VectorXd p_prev = VectorXd::Zero(2*st_params.num_segments);
    MatrixXd J;
    Vector3d ddx_des;
    VectorXd tau_ref;
};