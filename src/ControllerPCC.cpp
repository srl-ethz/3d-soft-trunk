//
// Created by yasu and rkk on 26/10/18.
//

#include "ControllerPCC.h"

/**
 * @brief implements a PID controller whose parameters are defined using the Ziegler-Nichols method.
 * @param Ku ultimate gain
 * @param period oscillation period (in seconds)
 * @return MiniPID controller
 */
MiniPID ZieglerNichols(double Ku, double period) {
    // https://en.wikipedia.org/wiki/Ziegler–Nichols_method
    double Kp = 0.45 * Ku;
    double Ki = 0; // Kp / (period/1.2) * CONTROL_PERIOD;
    double Kd = 0.;
    return MiniPID(Kp, Ki, Kd);
}

ControllerPCC::ControllerPCC() {
    std::cout << "ControllerPCC created...\n";
    // set up PID controllers
    // miniPIDs.resize(st_params::num_segments*2);
    if (st_params::controller == ControllerType::pid) {
        for (int j = 0; j < st_params::num_segments * 2; ++j)
            miniPIDs.push_back(MiniPID{st_params::pid_p[j], 0, 0});
    }
    // @todo probably used PD controllers for phi in original parameterization

    assert(st_params::num_segments == 2);
    K = VectorXd::Zero(st_params::num_segments * 2);
    D = VectorXd::Zero(st_params::num_segments * 2);
    for (int i = 0; i < st_params::num_segments; ++i) {
        K_p[i] = st_params::k_p[i];
        K_d[i] = st_params::k_d[i];
        if (st_params::parametrization == ParametrizationType::phi_theta) {
            K[2 * i + 1] = st_params::k[i];
            D[2 * i + 1] = st_params::beta[i]; // D[2*i] is set inside control loop
        }
    }

    alpha = VectorXd::Zero(st_params::num_segments * 2);
    A_f2p = MatrixXd::Zero(4, 2);
    A_p2f = MatrixXd::Zero(2, 4);
    A_p2f_all = MatrixXd::Zero(2 * st_params::num_segments, 4 * st_params::num_segments);

    A_f2p << 0.5, 0., 0., 0.5, -0.5, 0., 0., -0.5;
    A_p2f << 1., 0., -1., 0., 0., 1., 0., -1.;
    for (int j = 0; j < st_params::num_segments; ++j) {
        A_p2f_all.block(2 * j, 4 * j, 2, 4) = A_p2f;
    }

    alpha(0) = 0.00988;
    alpha(1) = alpha(0);
    alpha(2) = 0.0076;
    alpha(3) = alpha(2);

    ara = std::make_unique<AugmentedRigidArm>();
    vc = std::make_unique<ValveController>();
    cc = std::make_unique<CurvatureCalculator>();

    control_thread = std::thread(&ControllerPCC::control_loop, this);
}

void ControllerPCC::set_ref(const VectorXd &q_ref,
                            const VectorXd &dq_ref,
                            const VectorXd &ddq_ref) {
    // assign to member variables
    // @todo use mutex
    this->q_ref = q_ref;
    this->dq_ref = dq_ref;
    this->ddq_ref = ddq_ref;
}

void ControllerPCC::updateBCG(const VectorXd &q, const VectorXd &dq) {
    ara->update(q, dq);
    // these conversions from m space to q space are described in the paper
    B = ara->Jm.transpose() * ara->B_xi * ara->Jm;
    C = ara->Jm.transpose() * ara->B_xi * ara->dJm;
    G = ara->Jm.transpose() * ara->G_xi;
    J = ara->Jxi * ara->Jm;
}


void ControllerPCC::actuate(VectorXd f) {
    VectorXd p_vectorized = VectorXd::Zero(2); /** @brief 2D vector that expresses net pressure for X&Y directions */
    VectorXd p_actual = VectorXd::Zero(4); /** @brief actual pressure output to each chamber */
    Matrix2d mat; /** @brief mapping matrix from p_vectorized to p_actual */
    double theta, phi;
    for (int segment = 0; segment < st_params::num_segments; ++segment) {
        // first calculate p_vectorized for each segment
        if (st_params::parametrization == ParametrizationType::phi_theta) {
            if (simulate)
                theta = q_ref(2 * segment + 1);
            else
                theta = q(2 * segment + 1);

            if (theta < PI / 36 or simulate)
                // one weird trick
                // sensor reading for phi is unstable when theta is small. In those cases, use the reference value for phi.
                phi = q_ref(2 * segment);
            else
                phi = q(2 * segment);

            if (theta < PI / 36)
                mat << 0, -sin(phi), 0, cos(phi);
            else
                mat << -cos(phi) * sin(theta), -sin(phi), -sin(phi) * sin(theta), cos(phi);
            // @todo there probably needs to be an inversion here??
            p_vectorized = mat * f.segment(2 * segment, 2);
        } else if (st_params::parametrization == ParametrizationType::longitudinal) {
            //  @todo this is unverified
            p_vectorized = (A_f2p * f.segment(2 * segment, 2)) / alpha(2 * segment);
        }

        for (int i = 0; i < 2; ++i) {
            // convert vectorized pressure to actual pressure
            // do for N-S pair and E-W pair
            p_actual[i + 0] = st_params::p_offset + p_vectorized[i] / 2;
            p_actual[i + 2] = st_params::p_offset - p_vectorized[i] / 2;
            double minimum = p_actual.minCoeff();
            if (minimum < 0) {
                p_actual[i + 0] -= minimum;
                p_actual[i + 2] -= minimum;
            }
            // do sanity checks
            // p_actual[i+0] - p_actual[i+2] should equal p_vectorized[i]
            // no element of p_actual should be below 0
            assert(p_actual[i + 0] >= 0);
            assert(p_actual[i + 2] >= 0);
            assert(p_actual[i + 0] - p_actual[i + 0] == p_vectorized[i]);
        }
        for (int i = 0; i < 4; ++i)
            vc->setSinglePressure(4 * segment + i, p_actual[i]);
    }
}

void ControllerPCC::control_loop() {
    fmt::print("starting control loop...\n");
    Rate r{30};
    VectorXd f;
    while (true) {
        // variables to save the measured values.
        if (use_feedforward or simulate) {
            // don't use the actual values, since it's doing feedforward control.
            q = q_ref;
            dq = dq_ref;
        } else {
            // get the current configuration from CurvatureCalculator.
            q = cc->q;
            dq = cc->dq;
        }
        for (int i = 0; i < st_params::num_segments; ++i) {
            // take care of pose-dependent parameters
            if (st_params::parametrization == ParametrizationType::phi_theta)
                D(2 * i) = st_params::beta[i] * q(2 * i + 1) * q(2 * i + 1);
        }
        updateBCG(q, dq);
        if (st_params::controller == ControllerType::dynamic)
            f = G + C * dq_ref + B * ddq_ref +
                K.asDiagonal() * q_ref + K_p.asDiagonal() * (q_ref - q) + D.asDiagonal() * dq_ref +
                K_d.asDiagonal() * (dq_ref - dq);
        else if (st_params::controller == ControllerType::pid) {
            for (int i = 0; i < 2 * st_params::num_segments; ++i) {
                f[i] = miniPIDs[i].getOutput(q[i], q_ref[i]);
            }
        }
        actuate(f);
        r.sleep();
    }
}