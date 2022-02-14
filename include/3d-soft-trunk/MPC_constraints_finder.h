#pragma once

#include "3d-soft-trunk/ControllerPCC.h"
#include <random>

class MPC_constraints_finder: public ControllerPCC{
    public:
        MPC_constraints_finder(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type);

    protected:
        int N_obs; 
        int N_tar; 
        int N_trials;
        int N_check;

        MatrixXd obstacles; 
        MatrixXd targets; 

    private:
        MatrixXd Rotx_1(double theta);
        MatrixXd Roty_1(double theta);
        MatrixXd ee_position_1(VectorXd thetax, VectorXd thetay, VectorXd length1, VectorXd length2);
        MatrixXd ee_position_2(VectorXd thetax, VectorXd thetay, VectorXd length1, VectorXd length2);

        bool check_inclusion(VectorXd q_low, VectorXd q_up);
};