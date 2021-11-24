//
// Created by yasu and rkk on 26/10/18.
// revamped by oliver in may 21
//

#include "3d-soft-trunk/ControllerPCC.h"



ControllerPCC::ControllerPCC(const SoftTrunkParameters st_params) : st_params_(st_params){
    assert(st_params_.is_finalized());
    // set appropriate size for each member
    state_.setSize(st_params_.q_size);
    state_prev_.setSize(st_params_.q_size);
    state_ref_.setSize(st_params_.q_size);
    p = VectorXd::Zero(3 * st_params_.num_segments);
    f = VectorXd::Zero(2 * st_params_.num_segments);

    filename = "defaultController_log";

    mdl = std::make_unique<Model>(st_params_);
    ste = std::make_unique<StateEstimator>(st_params_);

    vc = std::make_unique<ValveController>("192.168.0.100", st_params_.valvemap, p_max);

    fmt::print("ControllerPCC object initialized.\n");
}

void ControllerPCC::set_ref(const srl::State &state_ref) {
    std::lock_guard<std::mutex> lock(mtx);
    // assign to member variables
    this->state_ref_ = state_ref;
    if (!is_initial_ref_received)
        is_initial_ref_received = true;
}

void ControllerPCC::set_ref(const Vector3d x_ref, const Vector3d &dx_ref, const Vector3d &ddx_ref){
    std::lock_guard<std::mutex> lock(mtx);
    Vector3d x_r = x_ref;
    if (x_ref.norm() > 0.27) {
        x_r = 0.27*x_ref.normalized(); //constrain reference position to be somewhat reachable
    }
    this->x_ref = x_r;
    this->dx_ref = dx_ref;
    this->ddx_ref = ddx_ref;
    if (!is_initial_ref_received)
        is_initial_ref_received = true;
}

void ControllerPCC::get_state(srl::State &state) {
    std::lock_guard<std::mutex> lock(mtx);
    state = this->state_;
}

void ControllerPCC::get_x(Vector3d &x) {
    std::lock_guard<std::mutex> lock(mtx);
    x = this->x;
}

void ControllerPCC::set_state(const srl::State &state) {
    std::lock_guard<std::mutex> lock(mtx);
    assert(st_params_.sensors[0] == SensorType::simulator);
    this->state_ = state;
}

void ControllerPCC::get_pressure(VectorXd& p){
    std::lock_guard<std::mutex> lock(mtx);
    p = this->p;
}

void ControllerPCC::toggleGripper(){
    gripperAttached = true;
    gripping = !gripping;
    vc->setSinglePressure(3*st_params_.num_segments, gripping*350); //350mbar to grip
}

VectorXd ControllerPCC::gravity_compensate(const srl::State state){
    assert(st_params_.sections_per_segment == 1);
    VectorXd gravcomp = dyn_.A_pseudo.inverse() * (dyn_.g + dyn_.K * state.q + dyn_.c);

    return gravcomp/100; //to mbar
}

void ControllerPCC::actuate(const VectorXd &p) { //actuates valves according to mapping from header
    assert(p.size() == 3 * st_params_.num_segments);
    for (int i = 0; i < 3*st_params_.num_segments; i++){
        vc->setSinglePressure(i, p(i));
    }
    if (logging){  
        log(state_.timestamp/10e6);                     //log once per control timestep
    }
}

std::vector<Eigen::Vector3d> ControllerPCC::get_objects(){
    bool has_qualisys = false;
    for (int i = 0; i < st_params_.sensors.size(); i++){
        if (st_params_.sensors[i] == SensorType::qualisys)
            has_qualisys = true;
    }
    assert(has_qualisys);

    return ste->get_objects;
}

void ControllerPCC::set_frequency(const double hz){
    assert(st_params_.sensors[0] == SensorType::simulator);
    this->dt = 1./hz;
}

void ControllerPCC::set_log_filename(const std::string s){
    this->filename = s;
}

bool ControllerPCC::simulate(const VectorXd &p){
    mdl->set_state(state_);
    mdl->get_dynamic_params(dyn_);
    state_prev_.ddq = state_.ddq;
    VectorXd p_adjusted = 100*p; //convert from mbar

    VectorXd b_inv_rest = dyn_.B.inverse() * (dyn_.A * p_adjusted - dyn_.c - dyn_.g - dyn_.K * state_.q);      //set up constant terms to not constantly recalculate
    MatrixXd b_inv_d = -dyn_.B.inverse() * dyn_.D;
    VectorXd ddq_prev;

    for (int i=0; i < int (dt/0.00001); i++){                                              //forward integrate dq with high resolution
        ddq_prev = state_.ddq;
        state_.ddq = b_inv_rest + b_inv_d*state_.dq;
        state_.dq += 0.00001*(2*(2*state_.ddq - ddq_prev) + 5*state_.ddq - ddq_prev)/6;  
    }

    state_.q = state_.q + state_.dq*dt + (dt*dt*(4*state_.ddq - state_prev_.ddq) / 6);


    if (logging){                                               //log once per control timestep
        log(t);
    }
    t+=dt;

    return !(abs(state_.ddq[0])>pow(10.0,10.0) or abs(state_.dq[0])>pow(10.0,10.0) or abs(state_.q[0])>pow(10.0,10.0)); //catches when the sim is crashing, true = all ok, false = crashing
}

void ControllerPCC::toggle_log(){
    if(!logging) {
        logging = true;
        if (!(st_params_.sensors[0] == SensorType::simulator)) {initial_timestamp = ste->get_timestamp();}
        else {initial_timestamp = 0;}
        this->filename = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, filename);
        fmt::print("Starting log to {}\n", this->filename);
        log_file.open(this->filename, std::fstream::out);
        log_file << "timestamp";

        //write header
        log_file << fmt::format(", x, y, z, x_ref, y_ref, z_ref, err");

        for (int i=0; i < st_params_.q_size; i++)
            log_file << fmt::format(", q_{}", i);
        for (int i=0; i < st_params_.num_segments*3; i++)
            log_file << fmt::format(", p_{}", i);

        log_file << "\n";
    } else {
        logging = false;
        fmt::print("Ending log to {}\n", this->filename);
        log_file.close();
    }
}

void ControllerPCC::log(double time){
    log_file << time;
    Vector3d x_tip;

    if (st_params_.sensors[0] == SensorType::qualisys){
        std::vector<Vector3d> x_qualisys;
        ste->get_x(x_qualisys);
        x_tip = x_qualisys[st_params_.num_segments];
    }

    log_file << fmt::format(", {}, {}, {}, {}, {}, {}, {}", x_tip(0), x_tip(1), x_tip(2), x_ref(0), x_ref(1), x_ref(2), (x_tip - x_ref).norm());

    for (int i=0; i < st_params_.q_size; i++)               //log q
        log_file << fmt::format(", {}", state_.q(i));
    for (int i=0; i < st_params_.num_segments*3; i++)
        log_file << fmt::format(", {}", p(i));
    log_file << "\n";
}