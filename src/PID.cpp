#include "3d-soft-trunk/PID.h"


PID::PID(CurvatureCalculator::SensorType sensor_type) : ControllerPCC::ControllerPCC(sensor_type){
    filename = "PID_log";

    for (int j = 0; j < st_params::num_segments; ++j){
            miniPIDs.push_back(ZieglerNichols(Ku[j], Tu[j], dt)); // for X direction
            miniPIDs.push_back(ZieglerNichols(Ku[j], Tu[j], dt)); // for Y direction
    }

    control_thread = std::thread(&PID::control_loop, this);
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
    srl::Rate r{1./dt};
    while(true){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        //update the internal visualization
        cc->get_curvature(state);
        stm->updateState(state);


        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;
        
        for (int i = 0; i < 2 * st_params::num_segments; ++i)
            f[i] = miniPIDs[i].getOutput(state.q[i], state_ref.q[i]);
        p = stm->pseudo2real(f + gravity_compensate(state));

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