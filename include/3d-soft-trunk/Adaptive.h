#pragma once

#include "3d-soft-trunk/ControllerPCC.h"
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
class Adaptive: public ControllerPCC {
public:
    Adaptive(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type = CurvatureCalculator::SensorType::qualisys, int objects = 0);
  
private:
    void control_loop();
    void avoid_singularity(srl::State state);
    MatrixXd computePinv(MatrixXd j, double e, double lambda);
    VectorXd Ka;
    VectorXd Kp;
    VectorXd Kd;
    MatrixXd J_inv;
    VectorXd aDot;
    VectorXd a;
    VectorXd tau;
    double eps;
    double lambda;
};