//
// Created by yasu and rkk on 26/10/18.
//

#include "3d-soft-trunk/ControllerPCC.h"



ControllerPCC::ControllerPCC(CurvatureCalculator::SensorType sensor_type) {

    
    extra_frames = 1;

    stm = std::make_unique<SoftTrunkModel>();
    // +X, +Y, -X, -Y
    std::vector<int> map = {1,2,5,3,6,4};
    vc = std::make_unique<ValveController>("192.168.0.100", map, p_max);
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

VectorXd ControllerPCC::gravity_compensate(srl::State state){
    assert(st_params::sections_per_segment == 1);
    VectorXd gravcomp = stm->A_pseudo.inverse() * (stm->g + stm->K * state.q + stm->D * state.dq);
    return gravcomp/100; //to mbar
}

void ControllerPCC::actuate(VectorXd f) { //actuates valves according to mapping from header
    f += 0*VectorXd::Ones(3*st_params::num_segments);
    for (int i = 0; i < 3*st_params::num_segments; i++){
        vc->setSinglePressure(i, f(i));
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

