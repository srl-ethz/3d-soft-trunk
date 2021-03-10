//
// Created by yasu and rkk on 26/10/18.
//

#include "3d-soft-trunk/ControllerPCC.h"

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

ControllerPCC::ControllerPCC(CurvatureCalculator::SensorType sensor_type) {
    // set up PID controllers
    if (st_params::controller == ControllerType::pid) {
        for (int j = 0; j < st_params::num_segments; ++j){
            miniPIDs.push_back(ZieglerNichols(Ku[j], Tu[j], dt)); // for X direction
            miniPIDs.push_back(ZieglerNichols(Ku[j], Tu[j], dt)); // for Y direction
        }
    }
    K_p = VectorXd::Zero(2*st_params::num_segments);
    K_p << 1, 1, 1, 1, 1, 1;

    K_d = VectorXd::Zero(2*st_params::num_segments);
    K_d << 1, 1, 1, 1, 1, 1;

    Pose pose;
    Pose pose_ref;

    p_vectorized = VectorXd::Zero(2 * st_params::num_segments);

    stm = std::make_unique<SoftTrunkModel>();
    // +X, +Y, -X, -Y
    std::vector<int> map = {0, 3, 2, 1, 4, 6, 7, 5, 11, 10, 8, 9};
    vc = std::make_unique<ValveController>("192.168.0.100", map, p_max);
    if (sensor_type == CurvatureCalculator::SensorType::bend_labs)
        cc = std::make_unique<CurvatureCalculator>(sensor_type, bendlabs_portname);
    else if (sensor_type == CurvatureCalculator::SensorType::qualisys)
        cc = std::make_unique<CurvatureCalculator>(sensor_type, qtm_address);

    control_thread = std::thread(&ControllerPCC::control_loop, this);
}

void ControllerPCC::set_ref(const Pose &pose_ref) {
    std::lock_guard<std::mutex> lock(mtx);
    assert(pose_ref.q.size() == st_params::num_segments * 2);
    assert(pose_ref.dq.size() == st_params::num_segments * 2);
    // assign to member variables
    this->pose_ref = pose_ref;
    if (!is_initial_ref_received)
        is_initial_ref_received = true;
}
void ControllerPCC::get_kinematic(Pose &pose) {
    std::lock_guard<std::mutex> lock(mtx);
    pose = this->pose;
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
    srl::Rate r{1./dt};
    VectorXd f = VectorXd::Zero(st_params::num_segments * 2);
    while (true) {
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);
        if (!is_initial_ref_received)
            continue;

        // first get the current pose
        if (use_feedforward or simulate) {
            // don't use the actual values, since it's doing feedforward control.
            pose = pose_ref;
        } else {
            // get the current configuration from CurvatureCalculator.
            cc->get_curvature(pose);
        }

        stm->updateState(pose);

        // calculate output
        if (st_params::controller == ControllerType::dynamic)
            f = stm->g + stm->c * dq_ref + stm->B * ddq_ref +
                stm->K.asDiagonal() * q_ref + K_p.asDiagonal() * (q_ref - q) + stm->D.asDiagonal() * dq_ref +
                K_d.asDiagonal() * (dq_ref - dq);
        else if (st_params::controller == ControllerType::pid) {
            for (int i = 0; i < 2 * st_params::num_segments; ++i)
                f[i] = miniPIDs[i].getOutput(pose.q[i], pose_ref.q[i]);
        }

        // actuate robot
        actuate(f);
    }
}
