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
    // use P-only control for now
    double Kp = 0.2 * Ku;
    double Ki = 0;//.4 * Ku / period * control_period;
    double Kd = 0;//.066 * Ku * period / control_period;
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

    chambermap << 1, -0.5, -0.5, 0, sqrt(3)/2, -sqrt(3)/2; 

    srl::State state;
    srl::State state_ref;


    stm = std::make_unique<SoftTrunkModel>();
    // +X, +Y, -X, -Y
    std::vector<int> map = {0, 1, 2, 8, 6, 5, 7, 3, 4};
    vc = std::make_unique<ValveController>("192.168.0.100", map, p_max);
    if (sensor_type == CurvatureCalculator::SensorType::bend_labs)
        cc = std::make_unique<CurvatureCalculator>(sensor_type, bendlabs_portname);
    else if (sensor_type == CurvatureCalculator::SensorType::qualisys)
        cc = std::make_unique<CurvatureCalculator>(sensor_type);

    control_thread = std::thread(&ControllerPCC::control_loop, this);
}

void ControllerPCC::set_ref(const srl::State &state_ref) {
    std::lock_guard<std::mutex> lock(mtx);
    assert(state_ref.q.size() == st_params::num_segments * 2);
    assert(state_ref.dq.size() == st_params::num_segments * 2);
    // assign to member variables
    this->state_ref = state_ref;
    if (!is_initial_ref_received)
        is_initial_ref_received = true;
}
void ControllerPCC::get_state(srl::State &state) {
    std::lock_guard<std::mutex> lock(mtx);
    state = this->state;
}
void ControllerPCC::get_pressure(VectorXd& p){
    std::lock_guard<std::mutex> lock(mtx);
    p = this->p;
}

VectorXd ControllerPCC::pseudo2real(VectorXd& pressure_pseudo){
    VectorXd output = VectorXd::Zero(3*st_params::num_segments);
    for (int i = 0; i < st_params::num_segments; i++){
        output.segment(3*i, 3) = chambermap.transpose()*(chambermap*chambermap.transpose()).inverse()*pressure_pseudo.segment(2*i, 2); //use Moore-Penrose to invert back onto real chambers
        double min_p = output.segment(3*i, 3).minCoeff();
        if (min_p < 0)
            output.segment(3*i, 3) -= min_p * Vector3d::Ones(); //remove any negative pressures, as they are not physically realisable
    }
    return output;
}

VectorXd ControllerPCC::gravity_compensate(srl::State state){
    /** @todo maybe add A_pseudo to SoftTrunkModel and use that + pseudo2real for simpler processing */
    VectorXd gravcomp = VectorXd::Zero(3 * st_params::num_segments);
    assert(st_params::sections_per_segment == 1);
    for (int i = 0; i < st_params::num_segments; i++){                  //calculate hold pressure for each PCC section by inverting stm.A blocks
        MatrixXd A_section = stm->A.block(2*i, 3*i, 2, 3); // section of A which corresponds to this segment
        gravcomp.segment(3*i,3) = A_section.transpose()*(A_section*A_section.transpose()).inverse()*(stm->g + segment_weight[i]*stm->K*state.q).segment(2*i, 2);
        double min_p = gravcomp.segment(3*i, 3).minCoeff();
        if (min_p < 0)
            gravcomp.segment(3*i, 3) -= min_p * Vector3d::Ones();
    }
    return gravcomp/100; //to mbar
}

void ControllerPCC::actuate(VectorXd f) { //actuates valves according to mapping from header
    for (int i = 0; i < 3*st_params::num_segments; i++){
        vc->setSinglePressure(i, f(i));
    }
}

void ControllerPCC::control_loop() {
    srl::Rate r{1./dt};
    VectorXd f = VectorXd::Zero(st_params::num_segments * 2); //2d pseudopressures
    while (true) {
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        // first get the current state from CurvatureCalculator.
        cc->get_curvature(state);
        stm->updateState(state);

        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;
        
        // calculate output
        switch (st_params::controller)
        {
        case ControllerType::dynamic:
            assert(false);
            // not implemented yet for new model
            break;
        case ControllerType::pid:
            for (int i = 0; i < 2 * st_params::num_segments; ++i)
                f[i] = miniPIDs[i].getOutput(state.q[i], state_ref.q[i]);
            p = pseudo2real(f) + gravity_compensate(state);
            break;
        case ControllerType::gravcomp:
            p = gravity_compensate(state);
            break;
        }

        // actuate robot
        actuate(p);
    }
}
