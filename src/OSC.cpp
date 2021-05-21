#include "3d-soft-trunk/OSC.h"

OSC::OSC(CurvatureCalculator::SensorType sensor_type, bool simulation, int objects) : ControllerPCC::ControllerPCC(sensor_type, simulation, objects){
    filename = "OSC_logger";

    //set the gains
    kp = 43.9;
    kd = 8.6;

    //OSC needs a higher refresh rate than other controllers
    dt = 1./100;

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
        //g_op = B_op*stm->J*stm->B.inverse()*stm->g;
        J_inv = stm->B.inverse()*stm->J.transpose()*B_op;
         
        f = B_op*ddx_ref;// + g_op;
        tau_null = -0.1*state.q*0;
        tau_ref = stm->J.transpose()*f + stm->g + stm->K * state.q + stm->D * state.dq + (MatrixXd::Identity(st_params::q_size, st_params::q_size) - stm->J.transpose()*J_inv.transpose())*tau_null;

        p = stm->pseudo2real(stm->A_pseudo.inverse()*tau_ref)/100;// + stm->pseudo2real(gravity_compensate(state));
        
        actuate(p);
    }
}

int OSC::singularity(const MatrixXd &J) {
    int order = 0;
    std::vector<Eigen::Vector3d> plane_normals(st_params::num_segments);            //normals to planes create by jacobian
    for (int i = 0; i < st_params::num_segments; i++) {
        Vector3d j1 = J.col(2*i).normalized();   //Eigen hates fun so we have to do this
        Vector3d j2 = J.col(2*i+1).normalized();
        plane_normals[i] = j1.cross(j2);
    }

    for (int i = 0; i < st_params::num_segments - 1; i++) {                         //check for singularities
        for (int j = 0; j < st_params::num_segments - 1 - i; j++){
            if (abs(plane_normals[i].dot(plane_normals[i+j+1])) > 0.9) order+=1;  //if the planes are more or less the same, we are near a singularity
        }
    }
    return order;
}