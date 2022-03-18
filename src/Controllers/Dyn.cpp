#include "3d-soft-trunk/Controllers/Dyn.h"

Dyn::Dyn(const SoftTrunkParameters st_params) : ControllerPCC::ControllerPCC(st_params){
    filename_ = "dynamic_log";
    Kp = 0.1*VectorXd::Ones(st_params.q_size);
    Kd = 0.000*VectorXd::Ones(st_params.q_size);
    dt_ = 1./100;

    control_thread_ = std::thread(&Dyn::control_loop, this);
}

void Dyn::control_loop(){
    srl::Rate r{1./dt_};
    while(run_){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);
        
        x_ = state_.tip_transforms[st_params_.num_segments+st_params_.prismatic].translation();
        
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;
        
        f_ = dyn_.A_pseudo.inverse() * (dyn_.D*state_ref_.dq 
                    + Kp.asDiagonal()*(state_ref_.q - state_.q) + Kd.asDiagonal()*(state_ref_.dq - state_.dq)); 
        p_ = mdl_->pseudo2real(f_/100) + mdl_->pseudo2real(gravity_compensate(state_)); //to mbar

        actuate(p_);
    }
};