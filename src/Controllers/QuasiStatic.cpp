#include "3d-soft-trunk/Controllers/QuasiStatic.h"

QuasiStatic::QuasiStatic(const SoftTrunkParameters st_params) : ControllerPCC::ControllerPCC(st_params){
    filename_ = "QS_logger";


    //set the gains
    kp = 10;
    kd = 5.5;


    //quasi static -> low refresh rate
    dt_ = 1./10;

    control_thread_ = std::thread(&QuasiStatic::control_loop, this);
}

void QuasiStatic::control_loop(){
    srl::Rate r{1./dt_};
    while(run_){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);
        
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;

        J = dyn_.J[st_params_.num_segments-1+st_params_.prismatic]; //tip jacobian

        x_ = state_.tip_transforms[st_params_.num_segments+st_params_.prismatic].translation();
        dx_ = J*state_.dq;
        
        ddx_des = ddx_ref_ + kp*(x_ref_ - x_).normalized()*0.05;            //desired acceleration from PD controller
        //normed to always assume a distance of 5cm


        for (int i = 0; i < singularity(J); i++){               //reduce jacobian order if the arm is in a singularity
            J.block(0,(st_params_.num_segments-1+st_params_.prismatic-i)*2,3,2) += 0.02*(i+1)*MatrixXd::Identity(3,2);
        }


        tau_ref = J.transpose()*ddx_des;
        VectorXd pxy = dyn_.A_pseudo.inverse()*tau_ref/10000;
        p_ = mdl_->pseudo2real(pxy + p_prev);
        p_prev += pxy;
        actuate(p_);
    }
}