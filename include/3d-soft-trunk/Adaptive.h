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
    void increase_gamma();
    void increase_delta();
    void increase_rate1();
    void increase_rate2();
    void increase_eps();   
    void decrease_kd();
    void decrease_kp();
    void decrease_gamma();
    void decrease_delta();
    void decrease_rate1();   
    void decrease_rate2(); 
    void decrease_eps();      
    void increase_damping();
    void decrease_damping();
    void increase_alpha();
    void decrease_alpha();
    void increase_stiffness(int seg);
    void decrease_stiffness(int seg); 
    void show_x();
    void change_ref1();
    void change_ref2();
    void change_ref3();
    void change_ref4();
    void start_AD();
    void start_ID();

    /** @brief log to memory and print to csv only after finishing 
     * @param time time for which will be logged
    */
    void toggle_fastlog(double time);


    VectorXd x_qualisys = VectorXd::Zero(3);
    VectorXd Ka = 0.0005*VectorXd::Ones(11);
    VectorXd a = VectorXd::Zero(11);
    VectorXd Kb = VectorXd::Ones(4);
    VectorXd b = VectorXd::Zero(4);    
    double rate1; //variation rate of estimates
    double rate2; //variation rate of estimates
    double gamma2;
    double sigma = 0.0;
    double dsigma = 0.0;
    double ddsigma = 0.0;
    double T = 30;        // final time changes depending on the timing law
    std::vector<Vector3d> target_points;
    
private:
    void control_loop();
    void avoid_drifting();
    void avoid_singularity(srl::State &state);
    void s_trapezoidal_speed(double t, double *sigma, double *dsigma, double *ddsigma, double *T);
    void Task_Circle_r2r(double sigma, double dsigma, double ddsigma);
    void Task_Linear_r2r(double sigma, double dsigma, double ddsigma);

    double sign(double val);
    VectorXd sat(VectorXd x, double delta);
    MatrixXd computePinv(MatrixXd J, double e, double lambda);
    VectorXd Ka_ = VectorXd::Zero(11);
    VectorXd Kb_ = VectorXd::Ones(4);
    VectorXd Kp = VectorXd::Zero(3);
    VectorXd Kd = VectorXd::Zero(3);   
    MatrixXd J_inv;
    MatrixXd Ainv;
    VectorXd aDot = VectorXd::Zero(11);
    VectorXd bDot = VectorXd::Zero(4);
    VectorXd a_min = 0.000001*VectorXd::Ones(11);
    VectorXd b_min = 0.000001*VectorXd::Ones(4);
    VectorXd a_max = 0.5*VectorXd::Ones(11);    
    VectorXd b_max = 0.01*VectorXd::Ones(4);      
    VectorXd tau = VectorXd::Zero(4);
    VectorXd s = VectorXd::Zero(4);   
    VectorXd s_d = VectorXd::Zero(4); //boundary layer manifold
    VectorXd e = VectorXd::Zero(3);
    VectorXd eDot = VectorXd::Zero(3);   
    VectorXd v = VectorXd::Zero(3);
    VectorXd vDot = VectorXd::Zero(3);       
    double eps;
    double lambda;
    double gamma;
    double delta; //boundary layer tickness
    double knd; //nullspace damping gain
    double alpha;
    double eps_custom; // for singularity
    double zz; // enable disable ID/AD

    VectorXd pprev = VectorXd::Zero(4);
    VectorXd d_pxy;
    double t;
    double t_internal = 0;
    bool fast_logging = false;
    MatrixXd log_matrix;


    Vector3d x_ref = Vector3d::Zero();
    Vector3d dx_ref = Vector3d::Zero();
    Vector3d ddx_ref = Vector3d::Zero();

    Vector3d p_i = Vector3d::Zero();
    Vector3d p_f = Vector3d::Zero();

    
    int target_point = 0;

    VectorXd x_tip1;
    VectorXd x_tip2;

    bool pause = true;

    //SRL logo
    void coordinates_S (double x_max = 0.12, double y_max = 0.12, double y_mid = 0.03, double default_z = -0.25);
    void coordinates_R (double x_max = 0.1, double y_max = 0.12, double y_mid = 0.03, double default_z = -0.25);
    void coordinates_L (double x_max = 0.12, double y_max = 0.12, double default_z = -0.25);
};