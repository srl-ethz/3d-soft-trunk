#include "3d-soft-trunk/Sensors/BendLabs.h"

BendLabs::BendLabs(const SoftTrunkParameters& st_params) : st_params_(st_params){
    assert (st_params_.is_finalized());

    state_ = st_params_.getBlankState();

    serialInterface = std::make_unique<SerialInterface>(st_params_.bendlabs_address,38400);
    calculatorThread = std::thread(&BendLabs::calculator_loop, this);

    fmt::print("BendLabs initialized on {}.\n", st_params_.bendlabs_address);
}

BendLabs::~BendLabs(){
    run = false;
    calculatorThread.join();
}


void BendLabs::calculator_loop(){
    std::fstream log_file;
    std::string filename = "bendlabs_log.csv";
    fmt::print("logging to {}\n", filename);
    log_file.open(filename, std::fstream::out);
    log_file << "timestamp";

    for (int i = 0; i < st_params_.q_size; ++i){
        log_file << fmt::format(", q_{}", i);
    }
    log_file << "\n";

    double frequency = 100.; //bendlabs cannot run faster than 100hz
    if (st_params_.sensor_refresh_rate < 100.){
        frequency = st_params_.sensor_refresh_rate;
    }

    unsigned long long int interval;
    state_prev_ = st_params_.getBlankState();

    srl::Rate r{frequency};

    while(run){
        r.sleep();
        timestamp_ += (int) (10e6 / frequency); //timestamp is tracked in us
        serialInterface->getData(bendLab_data_);

        bool same_as_prev = true;
        for (int i = 0; i < bendLab_data_.size(); i++){
            if (bendLab_data_[i] != bendLab_data_prev_[i]) {
                same_as_prev = false;
            }
        }

        if (same_as_prev){  //assumption that no two datapoints will be same due to noise
            continue;       //therefore if two concurrent datapoints are equal, there has been no update
        }

        interval = timestamp_ - last_timestamp_;
        last_timestamp_ = timestamp_;

        mtx.lock();

        for (int i = 0; i < st_params_.num_segments; i++){
            switch (st_params_.coord_type){
                case CoordType::thetax:
                    state_.q(2*i+st_params_.prismatic) = bendLab_data_[2*i+1]; //idk why they're mixed up
                    state_.q(2*i+1+st_params_.prismatic) = bendLab_data_[2*i+0];
                    break;
                case CoordType::phitheta:
                    state_.q(2*i+st_params_.prismatic) = atan2(bendLab_data_[2*i],bendLab_data_[2*i+1]);
                    state_.q(2*i+1+st_params_.prismatic) = sqrt(pow(bendLab_data_[2*i],2) + pow(bendLab_data_[2*i+1],2));
                    break;
            }
        }
        if (st_params_.prismatic){
            state_.q(0) = 0;
        }

        state_.dq = (state_.q - state_prev_.q)*0; //bendlabs has no way of logging time, so set speeds to 0
        state_.ddq = (state_.dq - state_prev_.dq)*0;
        state_prev_ = state_;
        state_.timestamp = timestamp_;

        mtx.unlock();

        log_file << timestamp_;
        for (int i = 0; i < 2 * st_params_.q_size; ++i){
            log_file << fmt::format(", {}", state_.q(i));
        }
        double phi, theta;
        log_file << "\n";
    }
    log_file.close();
}