#include "3d-soft-trunk/StateEstimator.h"

StateEstimator::StateEstimator(const SoftTrunkParameters& st_params) : st_params_(st_params), sensors_(st_params.sensors), filter_type_(st_params.filter_type) {
    assert(st_params.is_finalized());
    state_ = st_params.getBlankState();
    all_states_.resize(sensors_.size());

    for (int i = 0; i < all_states_.size(); i++){
        all_states_[i] = st_params_.getBlankState();
    }

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

StateEstimator::~StateEstimator(){
}

void StateEstimator::poll_sensors(){    
    get_states();
    get_filtered_state();
}

void StateEstimator::get_states(){
    for (int i = 0; i < all_states_.size(); i++)
    switch (sensors_[i]){
        case SensorType::qualisys:
            all_states_[i] = mocap_->state_;
            assert(all_states_[i].coordtype==st_params_.coord_type);
            break;
        case SensorType::bendlabs:
            all_states_[i] = bendlabs_->state_;
            assert(all_states_[i].coordtype==st_params_.coord_type);
            break;
        case SensorType::simulator:
            break;
    }
}

void StateEstimator::get_filtered_state(){
    /** depending on filter complexity, make an object to poll for filter output and hand it all states */
    
    switch (filter_type_){
        case FilterType::none: 
            this->state_ = this->all_states_[0];
            break;
    }
}