//
// Created by yasu and rkk on 26/10/18.
// revamped by oliver in may 21
//

#include "3d-soft-trunk/ControllerPCC.h"



ControllerPCC::ControllerPCC(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects) : st_params(st_params), sensor_type(sensor_type), objects(objects){
    assert(st_params.is_finalized());
    // set appropriate size for each member
    state.setSize(st_params.q_size);
    state_prev.setSize(st_params.q_size);
    state_ref.setSize(st_params.q_size);
    p = VectorXd::Zero(3 * st_params.num_segments);
    f = VectorXd::Zero(2 * st_params.num_segments);

    filename = "defaultController_log";
    stm = std::make_unique<SoftTrunkModel>(st_params);
    lag = std::make_unique<Lagrange>(st_params);
    // +X, +Y, -X, -Y
    std::vector<int> map = {3,1,2,4,6,5,0};
    
    if (sensor_type != CurvatureCalculator::SensorType::simulator) vc = std::make_unique<ValveController>("192.168.0.100", map, p_max);

    if (sensor_type == CurvatureCalculator::SensorType::bend_labs)
        cc = std::make_unique<CurvatureCalculator>(st_params, sensor_type, bendlabs_portname);
    else if (sensor_type == CurvatureCalculator::SensorType::qualisys) {
        cc = std::make_unique<CurvatureCalculator>(st_params, sensor_type, "" , objects);
        base_transform = cc->get_frame(0);
    }
    fmt::print("ControllerPCC object initialized.\n");
}

void ControllerPCC::set_ref(const srl::State &state_ref) {
    std::lock_guard<std::mutex> lock(mtx);
    // assign to member variables
    this->state_ref = state_ref;
    if (!is_initial_ref_received)
        is_initial_ref_received = true;
}

void ControllerPCC::set_ref(const Vector3d x_ref, const Vector3d &dx_ref, const Vector3d &ddx_ref){
    std::lock_guard<std::mutex> lock(mtx);
    Vector3d x_r = x_ref;
    if (x_ref.norm() > 0.27) {
        x_r = 0.27*x_ref.normalized(); //What is this???
    }
    this->x_ref = x_r;
    this->dx_ref = dx_ref;
    this->ddx_ref = ddx_ref;
    if (!is_initial_ref_received)
        is_initial_ref_received = true;
}

void ControllerPCC::get_state(srl::State &state) {
    std::lock_guard<std::mutex> lock(mtx);
    state = this->state;
}

void ControllerPCC::get_x(Vector3d &x) {
    std::lock_guard<std::mutex> lock(mtx);
    x = this->x;
}

void ControllerPCC::set_state(const srl::State &state) {
    std::lock_guard<std::mutex> lock(mtx);
    assert(sensor_type == CurvatureCalculator::SensorType::simulator);
    this->state = state;
}

void ControllerPCC::get_pressure(VectorXd& p){
    std::lock_guard<std::mutex> lock(mtx);
    p = this->p;
}

void ControllerPCC::toggleGripper(){
    gripperAttached = true;
    gripping = !gripping;
    vc->setSinglePressure(3*st_params.num_segments, gripping*200);
}

VectorXd ControllerPCC::gravity_compensate(const srl::State state){
    assert(st_params.sections_per_segment == 1);
    VectorXd gravcomp = stm->A_pseudo.inverse() * (stm->g + stm->K * state.q);

    return gravcomp/100; //to mbar
}

void ControllerPCC::actuate(const VectorXd &p) { //actuates valves according to mapping from header
    assert(p.size() == 3 * st_params.num_segments);
    for (int i = 0; i < 3*st_params.num_segments; i++){
        vc->setSinglePressure(i, p(i));
    }
    if (logging){  
        log((cc->get_timestamp() - initial_timestamp)/10e5);                     //log once per control timestep
    }
}

std::vector<Eigen::Vector3d> ControllerPCC::get_objects(){
    assert(sensor_type == CurvatureCalculator::SensorType::qualisys);
    std::vector<Eigen::Vector3d> object_vec(objects); 
    for (int i = 0; i < objects; i++) {
        object_vec[i] = cc->get_frame(st_params.num_segments + 1 + i).translation(); //read in the extra frame
        object_vec[i] = object_vec[i] - cc->get_frame(0).translation(); //change from global qualisys coordinates to relative to base
        object_vec[i] = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*object_vec[i]; //rotate to match the internal coordinates*/
    }
    
    return object_vec;
}

void ControllerPCC::set_frequency(const double hz){
    assert(sensor_type == CurvatureCalculator::SensorType::simulator);
    this->dt = 1./hz;
}

void ControllerPCC::newChamberConfig(Vector3d &angles) {
    stm->newChamberConfig(angles);
}

void ControllerPCC::set_log_filename(const std::string s){
    this->filename = s;
}

bool ControllerPCC::simulate(const VectorXd &p){
    stm->updateState(state);
    state_prev.ddq = state.ddq;
    VectorXd p_adjusted = 100*p; //convert from mbar

    VectorXd b_inv_rest = stm->B.inverse() * (stm->A * p_adjusted - stm->c - stm->g - stm->K * state.q);      //set up constant terms to not constantly recalculate
    MatrixXd b_inv_d = -stm->B.inverse() * stm->D;
    VectorXd ddq_prev;

    for (int i=0; i < int (dt/0.00001); i++){                                              //forward integrate dq with high resolution
        ddq_prev = state.ddq;
        state.ddq = b_inv_rest + b_inv_d*state.dq;
        state.dq += 0.00001*(2*(2*state.ddq - ddq_prev) + 5*state.ddq - ddq_prev)/6;  
    }

    state.q = state.q + state.dq*dt + (dt*dt*(4*state.ddq - state_prev.ddq) / 6);


    if (logging){                                               //log once per control timestep
        log(t);
    }
    t+=dt;

    return !(abs(state.ddq[0])>pow(10.0,10.0) or abs(state.dq[0])>pow(10.0,10.0) or abs(state.q[0])>pow(10.0,10.0)); //catches when the sim is crashing, true = all ok, false = crashing
}

void ControllerPCC::toggle_log(){
    if(!logging) {
        logging = true;
        if (sensor_type != CurvatureCalculator::SensorType::simulator) {initial_timestamp = cc->get_timestamp();}
        else {initial_timestamp = 0;}
        this->filename = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, filename);
        fmt::print("Starting log to {}\n", this->filename);
        log_file.open(this->filename, std::fstream::out);
        log_file << "experiment nr, timestamp";

        //write header
        log_file << fmt::format(", x, y, z, x_ref, y_ref, z_ref, err");

        for (int i=0; i < st_params.q_size; i++)
            log_file << fmt::format(", q_{}", i);
        for (int i=0; i < st_params.num_segments*3; i++)
            log_file << fmt::format(", p_{}", i);

        log_file << "\n";
    } else {
        logging = false;
        fmt::print("Ending log to {}\n", this->filename);
        log_file.close();
    }
}

void ControllerPCC::log(double time){
    log_file << experiment << ", ";
    log_file << time;


    VectorXd x_tip = stm->get_H(st_params.num_segments - 1).translation();
    if (sensor_type == CurvatureCalculator::SensorType::qualisys) x_tip = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params.num_segments).translation()-cc->get_frame(0).translation());
    
    log_file << fmt::format(", {}, {}, {}, {}, {}, {}, {}", x_tip(0), x_tip(1), x_tip(2), x_ref(0), x_ref(1), x_ref(2), (x_tip - x_ref).norm());
    for (int i=0; i < st_params.q_size; i++)               //log q
        log_file << fmt::format(", {}", state.q(i));
    for (int i=0; i < st_params.num_segments*3; i++)
        log_file << fmt::format(", {}", p(i));
    log_file << "\n";
}

