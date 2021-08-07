#include "3d-soft-trunk/Adaptive.h"

Adaptive::Adaptive(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects) : ControllerPCC(st_params,sensor_type,objects){
    filename = "adaptive_log";

    dt = 1./100;
    k_a << 1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.;
    k_p << 1.,1.,1.,1.;

    control_thread = std::thread(&Adaptive::control_loop, this);
}


void Adaptive::control_loop(){
    srl::Rate r{1./dt};
    while(true){
        r.sleep();
        if (sensor_type != CurvatureCalculator::SensorType::simulator) cc->get_curvature(state);
        lag->update(state, state_r);

        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;

        //ADD CONTROL LAW HERE, OUTPUT INTO TAU

        p = stm->pseudo2real(stm->A_pseudo.inverse()*tau);
        actuate(p);
    }
}