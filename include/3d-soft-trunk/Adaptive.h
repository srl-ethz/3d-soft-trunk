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
    void increase_kd();
    void increase_kp();
    void increase_gamma1();
    void increase_gamma2();
    void increase_delta();
    void increase_rate();
    void increase_eps();   
    void increase_lambda(); 
    void decrease_kd();
    void decrease_kp();
    void decrease_gamma1();
    void decrease_gamma2();
    void decrease_delta();
    void decrease_rate();   
    void decrease_eps();   
    void decrease_lambda();    
    void increase_damping();
    void decrease_damping();
    void increase_stiffness(int seg);
    void decrease_stiffness(int seg); 
    void show_x();
    void change_ref();
    VectorXd x_qualiszs = VectorXd::Zero(3);
    VectorXd Ka = VectorXd::Ones(11);
    VectorXd a = VectorXd::Zero(11);

private:
    void control_loop();
    void avoid_drifting();
    void avoid_singularity(srl::State &state);
    double sign(double val);
    VectorXd sat(VectorXd x, double delta);
    MatrixXd computePinv(MatrixXd J, double e, double lambda);
    VectorXd Ka_ = VectorXd::Ones(11);
    VectorXd Kp = VectorXd::Zero(3);
    VectorXd Kd = VectorXd::Zero(3);   
    MatrixXd J_inv;
    MatrixXd Ainv;
    VectorXd aDot = VectorXd::Zero(11);
    VectorXd a_min = VectorXd::Zero(11);
    VectorXd a_max = 0.3*VectorXd::Ones(11);    
    VectorXd tau = VectorXd::Zero(4);
    VectorXd s = VectorXd::Zero(4);   
    VectorXd s_d = VectorXd::Zero(4); //boundary layer manifold
    double eps;
    double lambda;
    double gamma1;
    double gamma2;
    double delta; //boundary layer tickness
    double rate; //variation rate of estimates
    double knd; //nullspace damping gain
};