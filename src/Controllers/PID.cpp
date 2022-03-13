#include "3d-soft-trunk/Controllers/PID.h"


PID::PID(const SoftTrunkParameters st_params) : ControllerPCC::ControllerPCC(st_params){
    filename_ = "PID_log";

    for (int j = 0; j < st_params.num_segments; ++j){
            miniPIDs.push_back(ZieglerNichols(Ku[j], Tu[j], dt_)); // for X direction
            miniPIDs.push_back(ZieglerNichols(Ku[j], Tu[j], dt_)); // for Y direction
    }

    control_thread_ = std::thread(&PID::control_loop, this);
}

MiniPID PID::ZieglerNichols(double Ku, double period, double control_period) {
    // https://en.wikipedia.org/wiki/Ziegler–Nichols_method
    // use P-only control for now
    double Kp = 0.2 * Ku;
    double Ki = 0;//.4 * Ku / period * control_period;
    double Kd = 0;//.066 * Ku * period / control_period;
    return MiniPID(Kp, Ki, Kd);
}

void PID::control_loop(){
    srl::Rate r{1./dt_};
    while(run_){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;
        
        for (int i = 0; i < 2 * st_params_.num_segments; ++i)
            f_[i] = miniPIDs[i].getOutput(state_.q[i], state_ref_.q[i]);
        
        p_ = mdl_->pseudo2real(f_ + gravity_compensate(state_));

        actuate(p_);
    }
}