//
// Created by yasu on 25/12/2020.
//

#include "ControllerPCC.h"

int main(){
    VectorXd q_ref = VectorXd::Zero(st_params::num_segments * 2);
    VectorXd dq_ref = VectorXd::Zero(st_params::num_segments * 2);
    VectorXd ddq_ref = VectorXd::Zero(st_params::num_segments * 2);
    for (int i = 0; i < st_params::num_segments; i++) {
        if (st_params::parametrization == ParametrizationType::phi_theta) {
            q_ref(2 * i) = 0.2;
            q_ref(2 * i + 1) = 0.5;
        } else if (st_params::parametrization == ParametrizationType::longitudinal) {
            q_ref(2 * i) = 0.15;
            q_ref(2 * i + 1) = 0.15;
        }
    }
    ControllerPCC cpcc{};
    cpcc.set_ref(q_ref, dq_ref, ddq_ref);

    Rate r{5};
    VectorXd q, dq, p_vectorized;
    MatrixXd J;
    while (true) {
        // the controller is updated at a much higher rate in a separate thread, this is just to show the output in realtime
        cpcc.get_kinematic(q, dq, J);
        cpcc.get_pressure(p_vectorized);
        fmt::print("--------\n");
        fmt::print("q_meas:\t{}\n", q.transpose());
        fmt::print("q_ref:\t{}\n", q_ref.transpose());
        fmt::print("p_vec:\t{}\n", p_vectorized.transpose());
        r.sleep();
    }

}