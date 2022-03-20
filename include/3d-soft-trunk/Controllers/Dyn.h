#pragma once

#include "3d-soft-trunk/ControllerPCC.h"
/** @brief A state space PD dynamic controller
 * @brief Used in Katzschmann 2019 Dynamic */
class Dyn: public ControllerPCC
{
public:
    Dyn(const SoftTrunkParameters st_params);

private:
    void control_loop();
    
    VectorXd Kp;
    VectorXd Kd;

};