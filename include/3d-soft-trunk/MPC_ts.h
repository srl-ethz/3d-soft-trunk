#pragma once

#include "3d-soft-trunk/ControllerPCC.h"
#include <casadi/casadi.hpp>
#include <casadi/core/optistack.hpp>
using namespace casadi;

class MPC_ts: public ControllerPCC{
    public:
        MPC_ts(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type);

    protected:

    private:
        void control_loop(); 
        Opti define_problem();
        void get_state_space(MatrixXd B, MatrixXd c, MatrixXd g, MatrixXd K, MatrixXd D, MatrixXd A, MatrixXd &sp_A, MatrixXd &sp_B, MatrixXd &sp_w, double Ts);
        MatrixXd matrix_exponential(MatrixXd A, int size); 
        MX Rotx(MX theta);
        MX Roty(MX theta);
        //MX ee_position(MX thetax, MX thetay, double length); 
        MX ee_position(MX thetax, MX thetay, MX length1, MX length2);
        MX axis_angle(MX thetax, MX thetay);

        VectorXd p_prev = VectorXd::Zero(2*st_params.num_segments);
        VectorXd tau_ref;

        int Horizon;
        Opti ctrl;
        bool solved; 
        //OptiSol sol; 

        MX q;
        MX q_dot; 
        MX A;  // state-space
        MX B;
        MX w;  // additional terms not in state-space
        MX u; 
        MX x_r; 
        MX q_0;
        MX q_dot_0;
        MX u_prev; 

        DM u_temp; // input placeholder
        MatrixXd p_temp; 
        VectorXd pxy;

        MatrixXd sp_A;
        MatrixXd sp_B;   // SS matrices
        MatrixXd sp_w; 

        DM sp_A_temp;
        DM sp_B_temp;
        DM sp_w_temp; 
        DM x_r_temp;  // conversion placeholders
        DM q_0_temp;
        DM q_dot_0_temp;
};