#pragma once

#include "ControllerPCC.h"

class OSC: public ControllerPCC
{
public:

    OSC(CurvatureCalculator::SensorType sensor_type);

    

private:
    
    void control_loop();

    /** @brief checks if the given jacobian is in a singularity
     *  @return order of singularity
     *  @details you can also use this as a bool
     */ 
    int singularity(const MatrixXd &J);

    /** @brief gains for OSC*/
    double kp;
    double kd;

    /** @brief operational dynamics */
    MatrixXd B_op;
    /** @brief operational space gravity*/
    VectorXd g_op;
    /** @brief extended jacobian inverse*/
    MatrixXd J_inv;

    /** @brief reference torques */
    VectorXd tau_ref;
    /** @brief nullspace torques */
    VectorXd tau_null;
    /** @brief reference acceleration in cartesian coordinates */
    Vector3d ddx_ref;
};