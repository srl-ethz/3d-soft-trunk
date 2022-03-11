#include "3d-soft-trunk/StateEstimator.h"

StateEstimator::StateEstimator(const SoftTrunkParameters& st_params) : st_params_(st_params), sensors_(st_params.sensors), filter_type_(st_params.filter_type) {
    assert(st_params.is_finalized());
    state_ = st_params.getBlankState();
    all_states_.resize(sensors_.size());
    for (int i = 0; i < sensors_.size(); i++){
        switch (sensors_[i]){
        case SensorType::qualisys:
            mocap_ = std::make_unique<MotionCapture>(st_params_);
            break;
        case SensorType::bendlabs:
            bendlabs_ = std::make_unique<BendLabs>(st_params_);
            break;
        case SensorType::simulator:
            break;
        }
    }

    fmt::print("State Estimator initialized with {} sensors.\n",sensors_.size());
}


void StateEstimator::poll_sensors(){    
    for (int i = 0; i < sensors_.size(); i++){
        get_state_from_ptr(all_states_[i],i);
    }
    state_ = get_filtered_state();
}

void StateEstimator::get_state_from_ptr(srl::State& state, int i){
    switch (sensors_[i]){
        case SensorType::qualisys:
            state = mocap_->state_;
            assert(state.coordtype==st_params_.coord_type);
            break;
        case SensorType::bendlabs:
            state_ = bendlabs_->state_;
            assert(state.coordtype==st_params_.coord_type);
            break;
        case SensorType::simulator:
            break;
    }
}

srl::State StateEstimator::get_filtered_state(){
    /** depending on filter complexity, make an object to poll for filter output and hand it all states */
    switch (filter_type_){
        default: 
            state_ = all_states_[0];
    }
}