#include "3d-soft-trunk/IKCon.h"

IKCon::IKCon(CurvatureCalculator::SensorType sensor_type, bool simulation, int objects) : ControllerPCC::ControllerPCC(sensor_type, simulation, objects){
    filename = "IK_logger";

    J_prev = MatrixXd::Zero(3, st_params::q_size);
    kp = 35;
    kd = 5.5;
    control_thread = std::thread(&IKCon::control_loop, this);
}

void IKCon::control_loop(){
    srl::Rate r{1./dt};
    while(true){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        //update the internal visualization
        if (!simulation) cc->get_curvature(state);
        
        stm->updateState(state);
        
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;

        J = stm->J[st_params::num_segments-1]; //tip jacobian
            dJ = (J - J_prev)/dt;

        J_prev = J;
        //do controls
        x = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params::num_segments).translation()-cc->get_frame(0).translation());
        //x = stm->get_H_base().rotation()*stm->get_H(st_params::num_segments-1).translation();

        dx = J*state.dq;
        ddx_ref = kp*(x_ref - x) + kd*(dx_ref - dx); 
        state_ref.ddq = J.completeOrthogonalDecomposition().pseudoInverse()*(ddx_ref - dJ*state.dq);

        tau_ref = stm->B*state_ref.ddq;
        
        p = stm->pseudo2real(stm->A_pseudo.inverse()*tau_ref/100) + stm->pseudo2real(gravity_compensate(state));

        if (!simulation) {actuate(p);}
        else {
            assert(simulate(p));
        }
    }

}