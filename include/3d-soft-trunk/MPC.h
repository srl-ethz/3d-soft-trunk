#pragma once

#include "3d-soft-trunk/ControllerPCC.h"
#include <casadi/casadi.hpp>
#include <casadi/core/optistack.hpp>
using namespace casadi;

class MPC: public ControllerPCC{
    public:
        MPC(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type);

    protected:

    private:
        void control_loop(); 
        Opti define_problem();
        void get_state_space(MatrixXd B, MatrixXd c, MatrixXd g, MatrixXd K, MatrixXd D, MatrixXd A, MatrixXd &sp_A, MatrixXd &sp_B, double Ts);

        VectorXd p_prev = VectorXd::Zero(2*st_params.num_segments);
        VectorXd tau_ref;

        int Horizon;
        Opti ctrl;
        //OptiSol sol; 

        MX A; 
        MX B;
        MX u; 
        MX q_r;
        MX q_dot_r; 

        DM u_temp; // input placeholder
        MatrixXd p_temp; 
        VectorXd pxy;

        MatrixXd sp_A;
        MatrixXd sp_B;   // SS matrices

        DM sp_A_temp;
        DM sp_B_temp;
        DM q_r_temp;
        DM q_dot_r_temp;  // conversion placeholders
};

