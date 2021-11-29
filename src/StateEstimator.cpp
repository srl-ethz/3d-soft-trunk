#include "3d-soft-trunk/StateEstimator.h"

StateEstimator::StateEstimator(const SoftTrunkParameters& st_params) : st_params_(st_params), sensors(st_params_.sensors), filter_type(st_params_.filter_type) {
    assert(st_params_.is_finalized());
    state_ = st_params_.getBlankState();
    states_.resize(sensors.size());
    for (int i = 0; i < sensors.size(); i++){
        switch (sensors[i]){
        case SensorType::qualisys:
            mocap = std::make_unique<MotionCapture>(st_params_);
            break;
        case SensorType::bendlabs:
            bendlabs = std::make_unique<BendLabs>(st_params_);
            break;
        case SensorType::simulator:
            break;
        }
    }

    polling_thread = std::thread(&StateEstimator::poll_sensors, this);
    fmt::print("State Estimator initialized with {} sensors.\n",sensors.size());
}

StateEstimator::~StateEstimator(){
    run = false;
    polling_thread.join();
}

void StateEstimator::get_state(srl::State& state){
    mtx.lock();
    state = this->state_;
    mtx.unlock();
}

void StateEstimator::get_states(std::vector<srl::State>& states){
    mtx.lock();
    states = this->states_;
    mtx.unlock();
}

void StateEstimator::poll_sensors(){
    srl::Rate r{st_params_.sensor_refresh_rate};
    while(run){
        r.sleep();
        for (int i = 0; i < sensors.size(); i++){
            get_state_from_ptr(states_[i],i);
        }
        state_ = get_filtered_state();
    }
}

void StateEstimator::get_state_from_ptr(srl::State& state, int i){
    switch (sensors[i]){
        case SensorType::qualisys:
            mocap->get_state(state);
            assert(state.coordtype==st_params_.coord_type);
            break;
        case SensorType::bendlabs:
            bendlabs->get_state(state);
            assert(state.coordtype==st_params_.coord_type);
            break;
        case SensorType::simulator:
            break;
    }
}

srl::State StateEstimator::get_filtered_state(){
    /** depending on filter complexity, make an object to poll for filter output and hand it all states */
    switch (filter_type){
        default: 
            return states_[0];
    }
}

/** @TODO: rewrite so it takes primary sensor instead of only qualisys */
Eigen::Transform<double, 3, Eigen::Affine> StateEstimator::get_object(int id){
    for (int i = 0; i < sensors.size(); i++){
        if (sensors[i] == SensorType::qualisys){
            return mocap->get_object(id);
        }
    }
    assert(false);
}

void StateEstimator::get_x(Vector3d &x){
    for (int i = 0; i < sensors.size(); i++) {
        if (sensors[i] == SensorType::qualisys){
            mocap->get_x(x);
        }
    }
}