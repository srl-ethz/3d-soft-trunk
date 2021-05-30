//
// Created by yasu and rkk on 26/10/18.
//

#include "3d-soft-trunk/ControllerPCC.h"



ControllerPCC::ControllerPCC(CurvatureCalculator::SensorType sensor_type, bool simulation, int objects) : simulation(simulation), extra_frames(objects){

    filename = "defaultController_log";

    stm = std::make_unique<SoftTrunkModel>();
    // +X, +Y, -X, -Y
    std::vector<int> map = {1,2,5,3,6,4};
    
    if (!simulation) vc = std::make_unique<ValveController>("192.168.0.100", map, p_max);

    if (sensor_type == CurvatureCalculator::SensorType::bend_labs)
        cc = std::make_unique<CurvatureCalculator>(sensor_type, bendlabs_portname);
    else if (sensor_type == CurvatureCalculator::SensorType::qualisys) {
        cc = std::make_unique<CurvatureCalculator>(sensor_type, "" , extra_frames);
        base_transform = cc->get_frame(0);
    }

}

void ControllerPCC::set_ref(const srl::State &state_ref) {
    std::lock_guard<std::mutex> lock(mtx);
    // assign to member variables
    this->state_ref = state_ref;
    if (!is_initial_ref_received)
        is_initial_ref_received = true;
}

void ControllerPCC::set_ref(const Vector3d &x_ref, const Vector3d &dx_ref){
    std::lock_guard<std::mutex> lock(mtx);
    this->x_ref = x_ref;
    this->dx_ref = dx_ref;
    if (!is_initial_ref_received)
        is_initial_ref_received = true;
}

void ControllerPCC::get_state(srl::State &state) {
    std::lock_guard<std::mutex> lock(mtx);
    state = this->state;
}

void ControllerPCC::set_state(const srl::State &state) {
    std::lock_guard<std::mutex> lock(mtx);
    assert(simulation);
    this->state = state;
}

void ControllerPCC::get_pressure(VectorXd& p){
    std::lock_guard<std::mutex> lock(mtx);
    p = this->p;
}


VectorXd ControllerPCC::gravity_compensate3(srl::State state){
    assert(st_params::sections_per_segment == 1);
    VectorXd gravcomp = VectorXd::Zero(3*st_params::num_segments);
    for (int i = 0; i < st_params::num_segments; i++){
        MatrixXd A_inverse_block = stm->A.block(2*i, 3*i, 2, 3).transpose()*(stm->A.block(2*i, 3*i, 2, 3)*stm->A.block(2*i, 3*i, 2, 3).transpose()).inverse();
        gravcomp.segment(3*i,3) = A_inverse_block * (stm->g + stm->K*state.q).segment(2*i,2);
        if (gravcomp.segment(3*i,3).minCoeff() < 0)
            gravcomp.segment(3*i, 3) -= gravcomp.segment(3*i,3).minCoeff() * Vector3d::Ones();
    }
    return gravcomp/100;
}

VectorXd ControllerPCC::gravity_compensate(const srl::State state){
    assert(st_params::sections_per_segment == 1);
    VectorXd gravcomp = stm->A_pseudo.inverse() * (stm->g + stm->K * state.q + stm->D * state.dq);
    return gravcomp/100; //to mbar
}

void ControllerPCC::actuate(const VectorXd &p) { //actuates valves according to mapping from header
    for (int i = 0; i < 3*st_params::num_segments; i++){
        vc->setSinglePressure(i, p(i));
    }
    if (logging){                                               //log once per control timestep
        log_file << t;
        for (int i=0; i < st_params::num_segments; i++){        //log tip pos
            VectorXd x_tip = stm->get_H(i).translation();
            log_file << fmt::format(", {}, {}, {}", x_tip(0), x_tip(1), x_tip(2));
        }
        for (int i=0; i < st_params::q_size; i++)               //log q
            log_file << fmt::format(", {}", state.q(i));
        log_file << "\n";
    }
}

std::vector<Eigen::Vector3d> ControllerPCC::get_objects(){
    std::vector<Eigen::Vector3d> objects(extra_frames); 
    for (int i = 0; i < extra_frames; i++) {
        objects[i] = cc->get_frame(st_params::num_segments + 1 + i).translation(); //read in the extra frame
        objects[i] = objects[i] - cc->get_frame(0).translation(); //change from global qualisys coordinates to relative to base
        objects[i] = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*objects[i]; //rotate to match the internal coordinates
    }
    
    return objects;
}

void ControllerPCC::set_frequency(const double hz){
    assert(simulation);
    this->dt = 1./hz;
}

void ControllerPCC::toggle_log(){
    if(!logging) {
        logging = true;
        initial_timestamp = cc->get_timestamp();
        this->filename = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, filename);
        fmt::print("Starting log to {}\n", this->filename);
        log_file.open(this->filename, std::fstream::out);
        log_file << "timestamp";

        //write header
        log_file << fmt::format(", x, y, z");

        for (int i=0; i < st_params::q_size; i++)
            log_file << fmt::format(", q_{}", i);

        log_file << "\n";
    } else {
        logging = false;
        fmt::print("Ending log to {}\n", this->filename);
        log_file.close();
    }
}

void ControllerPCC::set_log_filename(const std::string s){
    this->filename = s;
}

bool ControllerPCC::simulate(const VectorXd &p){
    stm->updateState(state);
    state_prev.ddq = state.ddq;

    VectorXd b_inv_rest = stm->B.inverse() * (stm->A * p - stm->c - stm->g - stm->K * state.q);      //set up constant terms to not constantly recalculate
    MatrixXd b_inv_d = -stm->B.inverse() * stm->D;
    VectorXd ddq_prev;

    for (int i=0; i < int (dt/0.00001); i++){                                              //forward integrate dq with high resolution
        ddq_prev = state.ddq;
        state.ddq = b_inv_rest + b_inv_d*state.dq;
        state.dq += 0.00001*(2*(2*state.ddq - ddq_prev) + 5*state.ddq - ddq_prev)/6;  
    }

    state.q = state.q + state.dq*dt + (dt*dt*(4*state.ddq - state_prev.ddq) / 6);


    if (logging){                                               //log once per control timestep
        log_file << t;
        for (int i=0; i < st_params::num_segments; i++){        //log tip pos
            VectorXd x_tip = stm->get_H(i).translation();
            log_file << fmt::format(", {}, {}, {}", x_tip(0), x_tip(1), x_tip(2));
        }
        for (int i=0; i < st_params::q_size; i++)               //log q
            log_file << fmt::format(", {}", state.q(i));
        log_file << "\n";
        
    }
    t+=dt;

    return !(abs(state.ddq[0])>pow(10.0,10.0) or abs(state.dq[0])>pow(10.0,10.0) or abs(state.q[0])>pow(10.0,10.0)); //catches when the sim is crashing, true = all ok, false = crashing

}
