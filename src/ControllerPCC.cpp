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
MiniPID ZieglerNichols(double Ku, double period, double control_period) {
    // https://en.wikipedia.org/wiki/Ziegler–Nichols_method
    double Kp = 0.2 * Ku;
    double Ki = 0.4 * Ku / period * control_period;
    double Kd = 0.066 * Ku * period / control_period;
    return MiniPID(Kp, Ki, Kd);
}

ControllerPCC::ControllerPCC(SensorType sensor_type) {
    assert(st_params::parametrization != ParametrizationType::phi_theta); /** @todo figure out how to deal with phi-theta parametrization and re-implemnt */
    // set up PID controllers
    if (st_params::controller == ControllerType::pid) {
        for (int j = 0; j < st_params::num_segments; ++j){
            miniPIDs.push_back(ZieglerNichols(Ku[j], Tu[j], dt)); // for X direction
            miniPIDs.push_back(ZieglerNichols(Ku[j], Tu[j], dt)); // for Y direction
        }
    }
    /** @todo probably used PD controllers for phi in original parameterization */
    /** @todo set up properly for different num_segments */
    K = VectorXd::Zero(2*st_params::num_segments);
    K << 1, 1, 1, 1, 1, 1;

    D = VectorXd::Zero(2*st_params::num_segments);
    D << 1, 1, 1, 1, 1, 1; // D[2*i] is set inside control loop

    K_p = VectorXd::Zero(2*st_params::num_segments);
    K_p << 1, 1, 1, 1, 1, 1;

    K_d = VectorXd::Zero(2*st_params::num_segments);
    K_d << 1, 1, 1, 1, 1, 1;

    q = VectorXd::Zero(2 * st_params::num_segments);
    dq = VectorXd::Zero(2 * st_params::num_segments);
    ddq = VectorXd::Zero(2 * st_params::num_segments);
    q_ref = VectorXd::Zero(2 * st_params::num_segments);
    dq_ref = VectorXd::Zero(2 * st_params::num_segments);
    ddq_ref = VectorXd::Zero(2 * st_params::num_segments);

    p_vectorized = VectorXd::Zero(2 * st_params::num_segments);

    ara = std::make_unique<AugmentedRigidArm>();
    // +X, +Y, -X, -Y
    std::vector<int> map = {0, 3, 2, 1, 4, 6, 7, 5, 11, 10, 8, 9};
    vc = std::make_unique<ValveController>("192.168.0.100", map, p_max);
    cc = std::make_unique<CurvatureCalculator>(sensor_type);

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

void ControllerPCC::updateBCGJ(const VectorXd &q, const VectorXd &dq) {
    ara->update(q, dq);
    // these conversions from m space to q space are described in the paper
    B = ara->Jm.transpose() * ara->B_xi * ara->Jm;
    C = ara->Jm.transpose() * ara->B_xi * ara->dJm;
    G = ara->Jm.transpose() * ara->G_xi;
    J = ara->Jxi * ara->Jm;
}

void ControllerPCC::get_kinematic(VectorXd &q, VectorXd dq, MatrixXd &J) {
    std::lock_guard<std::mutex> lock(mtx);
    q = this->q;
    dq = this->dq;
    J = this->J;
}
void ControllerPCC::get_dynamic(MatrixXd &B, MatrixXd &C, MatrixXd &G){
    std::lock_guard<std::mutex> lock(mtx);
    B = this->B;
    C = this->C;
    G = this->G;
}
void ControllerPCC::get_pressure(VectorXd& p_vectorized){
    std::lock_guard<std::mutex> lock(mtx);
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
            /** @todo I am seriously concerned about how this behaves near q = 0 */
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
            p_vectorized_segment /= alpha; // convert force to pressure
        p_vectorized.segment(2 * segment, 2) = p_vectorized_segment; // save to p_vectorized

        for (int i = 0; i < 2; ++i) {
            // convert vectorized pressure to actual pressure
            // do for N-S pair and E-W pair
            p_actual[i + 0] = p_offset + p_vectorized_segment[i] / 2;
            p_actual[i + 2] = p_offset - p_vectorized_segment[i] / 2;
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
            /** @todo do fuzzier evaluation */
        }
        for (int i = 0; i < 4; ++i)
            vc->setSinglePressure(4 * segment + i, p_actual[i]);
    }
}

void ControllerPCC::control_loop() {
    Rate r{1./dt};
    VectorXd f = VectorXd::Zero(st_params::num_segments * 2);
    while (true) {
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);
        if (!is_initial_ref_received)
            continue;

        // first get the current pose
        if (use_feedforward or simulate) {
            // don't use the actual values, since it's doing feedforward control.
            q = q_ref;
            dq = dq_ref;
        } else {
            // get the current configuration from CurvatureCalculator.
            cc->get_curvature(q, dq, ddq);
        }

        // update rigid model and calculate params from it
        updateBCGJ(q, dq);

        // take care of pose-dependent parameters
        for (int i = 0; i < st_params::num_segments; ++i) {
            if (st_params::parametrization == ParametrizationType::phi_theta)
                D(2 * i) = beta * q(2 * i + 1) * q(2 * i + 1);
        }

        // calculate output
        if (st_params::controller == ControllerType::dynamic)
            f = G + C * dq_ref + B * ddq_ref +
                K.asDiagonal() * q_ref + K_p.asDiagonal() * (q_ref - q) + D.asDiagonal() * dq_ref +
                K_d.asDiagonal() * (dq_ref - dq);
        else if (st_params::controller == ControllerType::pid) {
            for (int i = 0; i < 2 * st_params::num_segments; ++i)
                f[i] = miniPIDs[i].getOutput(q[i], q_ref[i]);
        }

        // actuate robot
        actuate(f);
    }
}