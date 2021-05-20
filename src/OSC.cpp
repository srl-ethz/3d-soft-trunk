#include "3d-soft-trunk/OSC.h"

OSC::OSC(CurvatureCalculator::SensorType sensor_type) : ControllerPCC::ControllerPCC(sensor_type){
    filename = "OSC_logger";

    kp = 43.9;
    kd = 8.6;

    control_thread = std::thread(&OSC::control_loop, this);
}

void OSC::control_loop() {
    srl::Rate r{1./dt};
    while(true){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        //update the internal visualization
        cc->get_curvature(state);
        stm->updateState(state);

        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;
        
        //do controls
        x = stm->ara->get_H_base().rotation()*stm->ara->get_H_tip().translation();
        dx = stm->J*state.dq;
        ddx_ref = kp*(x_ref - x) + kd*(dx_ref - dx);            //values are critically damped approach
        B_op = (stm->J*stm->B.inverse()*stm->J.transpose()).inverse();
        g_op = B_op*stm->J*stm->B.inverse()*stm->g;
        J_inv = stm->B.inverse()*stm->J.transpose()*B_op;
         
        f = B_op*ddx_ref;// + g_op;
        tau_null = -0.1*state.q*0;
        tau_ref = stm->J.transpose()*f /*+ stm->K * state.q*/ + stm->D * state.dq + (MatrixXd::Identity(st_params::q_size, st_params::q_size) - stm->J.transpose()*J_inv.transpose())*tau_null;

        p = /*stm->pseudo2real(stm->A_pseudo.inverse()*tau_ref)/100 +*/ stm->pseudo2real(gravity_compensate(state));
        
        if (logging) {
            log_file << (cc->get_timestamp() - initial_timestamp)/ 1.0e6;
            
            log_file << fmt::format(", {}, {}, {}", x(0), x(1), x(2));
            
            for (int i=0; i < st_params::q_size; i++)               //log q
                log_file << fmt::format(", {}", state.q(i));
            log_file << "\n";
        }
    actuate(p);

    }
}