#pragma once

#include "ControllerPCC.h"

class Dyn: public ControllerPCC
{
public:
    Dyn(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType, bool simulation = false);

private:
    void control_loop();
    
    VectorXd Kp;
    VectorXd Kd;

};