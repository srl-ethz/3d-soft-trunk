#include "3d-soft-trunk/Dyn.h"

Dyn::Dyn(CurvatureCalculator::SensorType sensor_type, bool simulation) : ControllerPCC::ControllerPCC(sensor_type, simulation){
    filename = "dynamic_log";
    Kp = 0.1*VectorXd::Ones(st_params::q_size);
    Kd = 0.000*VectorXd::Ones(st_params::q_size);
    dt = 1./100;
    control_thread = std::thread(&Dyn::control_loop, this);
}

void Dyn::control_loop(){
    srl::Rate r{1./dt};
    while(true){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        //update the internal visualization
        if (!simulation) cc->get_curvature(state);
        
        stm->updateState(state);
        x = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params::num_segments).translation()-cc->get_frame(0).translation());
        
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;
        
        f = stm->A_pseudo.inverse() * (stm->D*state_ref.dq 
                    + Kp.asDiagonal()*(state_ref.q - state.q) + Kd.asDiagonal()*(state_ref.dq - state.dq)); 
        p = stm->pseudo2real(f/100) + stm->pseudo2real(gravity_compensate(state)); //to mbar

        if (!simulation) actuate(p);
        else simulate(p);

    }
};