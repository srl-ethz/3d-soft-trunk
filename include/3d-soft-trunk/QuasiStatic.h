#pragma once

#include "3d-soft-trunk/ControllerPCC.h"
#include <casadi/casadi.hpp>
#include <casadi/core/optistack.hpp>
using namespace casadi;

class QuasiStatic: public ControllerPCC{
public:
    QuasiStatic(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects = 0);
    struct optimal_solution
    {
        VectorXd pressure;
        OptiSol solution;
    };


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


    Opti define_problem();
    QuasiStatic::optimal_solution pressure_finder(VectorXd torque, MatrixXd A_real); 
    QuasiStatic::optimal_solution pressure_finder_warm(VectorXd torque, MatrixXd A_real, DM old_sol);
    Opti ctrl; 
    MX A;
    MX tau;
    MX u; 
    MatrixXd chamberMatrix = MatrixXd::Zero(2,3);
    MatrixXd mapping_matrix = MatrixXd::Zero(4,6);
    bool solved; 
    
};