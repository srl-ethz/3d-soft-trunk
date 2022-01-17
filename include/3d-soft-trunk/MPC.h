#pragma once

#include "3d-soft-trunk/ControllerPCC.h"
#include <casadi/casadi.hpp>
#include <casadi/core/optistack.hpp>
using namespace casadi;

class MPC: public ControllerPCC{
    public:
        MPC(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type);
        void set_ref(const srl::State &state_ref1, const srl::State &state_ref2, int edge);
        const int Horizon = 10;

    protected:
        MatrixXd q_ref_long = MatrixXd::Zero(st_params.q_size, Horizon+1); 

    private:
        void control_loop(); 
        Opti define_problem();
        void get_state_space(MatrixXd B, MatrixXd c, MatrixXd g, MatrixXd K, MatrixXd D, MatrixXd A, MatrixXd &sp_A, MatrixXd &sp_B, MatrixXd &sp_w, double Ts);
        MatrixXd matrix_exponential(MatrixXd A, int size); 

        VectorXd p_prev = VectorXd::Zero(2*st_params.num_segments);
        VectorXd tau_ref;

        Opti ctrl;
        bool solved; 
        //OptiSol sol; 

        MX q;
        MX q_dot; 
        MX A;  // state-space
        MX B;
        MX w;  // additional terms not in state-space
        MX u; 
        MX q_r;
        MX q_dot_r; 
        MX q_0;
        MX q_dot_0;
        MX u_prev; 

        DM u_temp; // input placeholder
        MatrixXd p_temp = MatrixXd::Zero(2*st_params.num_segments,1); 
        VectorXd pxy;

        MatrixXd sp_A = MatrixXd::Zero(2*st_params.q_size, 2*st_params.q_size);
        MatrixXd sp_B = MatrixXd::Zero(2*st_params.q_size, 2*st_params.num_segments);   // SS matrices
        MatrixXd sp_w = MatrixXd::Zero(2*st_params.q_size, 1); 

        DM sp_A_temp = DM::nan(2*st_params.q_size, 2*st_params.q_size);
        DM sp_B_temp = DM::nan(2*st_params.q_size, 2*st_params.num_segments);
        DM sp_w_temp = DM::nan(2*st_params.q_size, 1); 
        //DM q_r_temp = DM::nan(st_params.q_size, 1);
        DM q_r_temp = DM::nan(st_params.q_size, Horizon+1);
        DM q_dot_r_temp = DM::nan(st_params.q_size, 1);  // conversion placeholders
        DM q_0_temp = DM::nan(st_params.q_size, 1);
        DM q_dot_0_temp = DM::nan(st_params.q_size, 1);
        DM q_0_large = DM::nan(st_params.q_size, Horizon+1);    // needed for warm-start
        DM q_dot_0_large = DM::nan(st_params.q_size, Horizon+1); 

        MatrixXd sp_A_c = MatrixXd::Zero(2*st_params.q_size, 2*st_params.q_size);
        MatrixXd sp_B_c = MatrixXd::Zero(2*st_params.q_size, 2*st_params.num_segments); 
        MatrixXd sp_w_c = MatrixXd::Zero(2*st_params.q_size, 1);
        MatrixXd Ad; 

        int counter = 0; 
        double total_time = 0;
        VectorXd slow_execution = VectorXd::Zero(5); 
};

