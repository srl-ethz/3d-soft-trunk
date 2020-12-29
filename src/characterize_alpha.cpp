//
// Created by yasu on 31/10/18.
//

#include "ControllerPCC.h"

int main() {
    // very preliminary and rough...
    // apply known force at tip, then robot is controlled to be straight with PID control.
    // by using final pressure & Jacobian, alpha (map from pressure to force can be calculated
    VectorXd q_ref = VectorXd::Zero(st_params::num_segments * 2);
    VectorXd dq_ref = VectorXd::Zero(st_params::num_segments * 2);
    VectorXd ddq_ref = VectorXd::Zero(st_params::num_segments * 2);
    ControllerPCC cpcc{};
    cpcc.set_ref(q_ref, dq_ref, ddq_ref);

    sleep(15);
    MatrixXd J;
    VectorXd q, dq, p_vectorized;
    cpcc.get_jacobian(J);
    cpcc.get_status(q, dq, p_vectorized);
    fmt::print("alpha * P = J^T f\n");
    fmt::print("P = {}\n", p_vectorized);
    fmt::print("J:\n{}\n", J);
    fmt::print("q: {}\n", q.transpose());
    return 1;
}