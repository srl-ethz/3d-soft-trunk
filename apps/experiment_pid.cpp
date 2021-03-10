//
// Created by yasu on 25/12/2020.
//

#include "3d-soft-trunk/ControllerPCC.h"

/**
 * @brief run a PID controller
 */
int main(){

    srl::State state_ref, state;
    fmt::print("state:{}\n", state_ref.q.transpose());

    for (int i = 0; i < st_params::num_segments; i++) {
        state_ref.q(2 * i) = 0.15;
        state_ref.q(2 * i + 1) = 0.15;
    }
    ControllerPCC cpcc{CurvatureCalculator::SensorType::qualisys};
    cpcc.set_ref(state_ref);
    srl::Rate r{5};
    VectorXd p_vectorized;
    while (true) {
        // the controller is updated at a much higher rate in a separate thread, this is just to show the output in realtime
        cpcc.get_kinematic(state);
        cpcc.get_pressure(p_vectorized);
        fmt::print("--------\n");
        fmt::print("q_meas:\t{}\n", state.q.transpose());
        fmt::print("q_ref:\t{}\n", state_ref.q.transpose());
        fmt::print("p_vec:\t{}\n", p_vectorized.transpose());
        r.sleep();
    }

}
