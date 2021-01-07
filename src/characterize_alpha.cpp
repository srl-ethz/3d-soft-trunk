//
// Created by yasu on 31/10/18.
//

#include "ControllerPCC.h"

int main() {
    // procedure for data-driven characterization of alpha.
    // apply known lateral force at tip.
    // robot is controlled to be straight with PID control.
    // by using final pressure & Jacobian, alpha (map from pressure to generalized force) can be calculated
    // alpha can also be calculated from the model, by using the area of chamber (A) & distance between the chamber and center axis (d).
    // e.g. for the longitudinal parametrization, tau [N] * 1 [m] = (100 P [mbar])[Pa] * A[m^2] * d[m]
    // alpha = A * d * 100
    // @todo update ControllerPCC to be able to set whether or not to do PID control
    VectorXd q_ref = VectorXd::Zero(st_params::num_segments * 2);
    VectorXd dq_ref = VectorXd::Zero(st_params::num_segments * 2);
    VectorXd ddq_ref = VectorXd::Zero(st_params::num_segments * 2);
    ControllerPCC cpcc{};
    cpcc.set_ref(q_ref, dq_ref, ddq_ref);

    sleep(15);
    MatrixXd J;
    VectorXd q, dq, p_vectorized;
    cpcc.get_kinematic(q, dq, J);
    cpcc.get_pressure(p_vectorized);
    fmt::print("alpha * P = J^T f\n");
    fmt::print("P = {}\n", p_vectorized);
    fmt::print("J:\n{}\n", J);
    fmt::print("q: {}\n", q.transpose());
    return 1;
}