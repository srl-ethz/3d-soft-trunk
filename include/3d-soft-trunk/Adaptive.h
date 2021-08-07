#pragma once

#include "SoftTrunk_common.h"
#include "3d-soft-trunk/ControllerPCC.h"

class Adaptive: public ControllerPCC {
public:
    Adaptive(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type = CurvatureCalculator::SensorType::qualisys, int objects = 0);
    
    
    VectorXd k_a;
    VectorXd k_p;

private:
    void control_loop();

    srl::State state_r;
};