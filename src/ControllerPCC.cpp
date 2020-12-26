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
    if (st_params::controller == ControllerType::pid) {
        for (int j = 0; j < st_params::num_segments * 2; ++j)
            miniPIDs.push_back(MiniPID{st_params::pid_p[j], 0, 0});
    }
    // @todo probably used PD controllers for phi in original parameterization

    assert(st_params::num_segments == 1);
    K = VectorXd::Zero(st_params::num_segments * 2);
    D = VectorXd::Zero(st_params::num_segments * 2);
    for (int i = 0; i < st_params::num_segments; ++i) {
        if (st_params::parametrization == ParametrizationType::phi_theta) {
            K[2 * i + 1] = st_params::k[i];
            D[2 * i + 1] = st_params::beta[i]; // D[2*i] is set inside control loop
        }
    }

    K_p = VectorXd::Zero(st_params::num_segments * 2);
    K_d = VectorXd::Zero(st_params::num_segments * 2);
    for (int i = 0; i < st_params::num_segments * 2; ++i) {
        K_p[i] = st_params::k_p[i];
        K_d[i] = st_params::k_d[i];
    }

    alpha = VectorXd::Zero(st_params::num_segments);

    alpha(0) = 0.0001;
    p_vectorized = VectorXd::Zero(2 * st_params::num_segments);
    q = VectorXd::Zero(2 * st_params::num_segments);
    dq = VectorXd::Zero(2 * st_params::num_segments);
    q_ref = VectorXd::Zero(2 * st_params::num_segments);
    dq_ref = VectorXd::Zero(2 * st_params::num_segments);
    ddq_ref = VectorXd::Zero(2 * st_params::num_segments);

    ara = std::make_unique<AugmentedRigidArm>();
    std::vector<int> map = {0, 1, 2, 3};//, 4, 5, 6, 7, 8, 9, 10, 11};
    vc = std::make_unique<ValveController>("192.168.0.100", map, 400);
    cc = std::make_unique<CurvatureCalculator>();

    control_thread = std::thread(&ControllerPCC::control_loop, this);
}

void ControllerPCC::set_ref(const VectorXd &q_ref,
                            const VectorXd &dq_ref,
                            const VectorXd &ddq_ref) {
    std::lock_guard<std::mutex> lock(mtx);
    assert(q_ref.size() == st_params::num_segments * 2);
    assert(dq_ref.size() == st_params::num_segments * 2);
    // assign to member variables
    this->q_ref = q_ref;
    this->dq_ref = dq_ref;
    this->ddq_ref = ddq_ref;
    if (!is_initial_ref_received)
        is_initial_ref_received = true;
}

void ControllerPCC::updateBCG(const VectorXd &q, const VectorXd &dq) {
    ara->update(q, dq);
    // these conversions from m space to q space are described in the paper
//    B = ara->Jm.transpose() * ara->B_xi * ara->Jm;
//    C = ara->Jm.transpose() * ara->B_xi * ara->dJm;
//    G = ara->Jm.transpose() * ara->G_xi;
//    J = ara->Jxi * ara->Jm;
}

void ControllerPCC::get_status(VectorXd &q, VectorXd dq, VectorXd &p_vectorized) {
    std::lock_guard<std::mutex> lock(mtx);
    q = this->q;
    dq = this->dq;
    p_vectorized = this->p_vectorized;
}


void ControllerPCC::actuate(VectorXd f) {
    VectorXd p_vectorized_segment = VectorXd::Zero(2); // part of p_vectorized, for each segment
    VectorXd p_actual = VectorXd::Zero(4); /** @brief actual pressure output to each chamber */
    Matrix2d mat; /** @brief mapping matrix from f to p_vectorized_segment */
    double theta, phi;
    for (int segment = 0; segment < st_params::num_segments; ++segment) {
        // first calculate p_vectorized_segment for each segment
        if (st_params::parametrization == ParametrizationType::phi_theta) {
            // @todo I am seriously concerned about how this behaves near q = 0
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
            if (phi < 0.1)
                phi = 0.1; // super hacky temporary solution
            if (false)//theta < PI / 36)
                mat << 0, -sin(phi), 0, cos(phi);
            else
                mat << -cos(phi) * sin(theta), -sin(phi), -sin(phi) * sin(theta), cos(phi);
            mat = mat.inverse().eval();
            fmt::print("mat inv:{}\n", mat);
            p_vectorized_segment = mat * f.segment(2 * segment, 2);
        } else if (st_params::parametrization == ParametrizationType::longitudinal)
            p_vectorized_segment = f.segment(2 * segment, 2);
        if (st_params::controller == ControllerType::dynamic)
            p_vectorized_segment /= alpha(segment); // convert force to pressure
        p_vectorized.segment(2 * segment, 2) = p_vectorized_segment; // save to p_vectorized

        for (int i = 0; i < 2; ++i) {
            // convert vectorized pressure to actual pressure
            // do for N-S pair and E-W pair
            p_actual[i + 0] = st_params::p_offset + p_vectorized_segment[i] / 2;
            p_actual[i + 2] = st_params::p_offset - p_vectorized_segment[i] / 2;
            double minimum = std::min(p_actual[i + 0], p_actual[i + 2]);
            if (minimum < 0) {
                p_actual[i + 0] -= minimum;
                p_actual[i + 2] -= minimum;
            }
            // do sanity checks
            // p_actual[i+0] - p_actual[i+2] should equal p_vectorized_segment[i]
            // no element of p_actual should be below 0
            assert(p_actual[i + 0] >= 0);
            assert(p_actual[i + 2] >= 0);
            // assert(p_actual[i + 0] - p_actual[i + 2] == p_vectorized_segment[i]);
            // @todo do fuzzier evaluation
        }
        for (int i = 0; i < 4; ++i)
            vc->setSinglePressure(4 * segment + i, p_actual[i]);
    }
}

void ControllerPCC::control_loop() {
    fmt::print("starting control loop...\n");
    Rate r{30};
    VectorXd f = VectorXd::Zero(st_params::num_segments * 2);
    VectorXd q, dq, ddq;
    fmt::print("starting loop...\n");
    while (true) {
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);
        if (!is_initial_ref_received)
            continue;

        // variables to save the measured values.
        if (use_feedforward or simulate) {
            // don't use the actual values, since it's doing feedforward control.
            q = q_ref;
            dq = dq_ref;
        } else {
            // get the current configuration from CurvatureCalculator.
            cc->get_curvature(q, dq, ddq);
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
    }
}