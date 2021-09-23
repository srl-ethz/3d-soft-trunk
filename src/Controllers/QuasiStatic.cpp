#include "3d-soft-trunk/Controllers/QuasiStatic.h"

QuasiStatic::QuasiStatic(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects) : ControllerPCC::ControllerPCC(st_params, sensor_type, objects){
    filename = "QS_logger";


    //set the gains
    kp = 10;
    kd = 5.5;


    //qs
    dt = 1./10;

    control_thread = std::thread(&QuasiStatic::control_loop, this);
}

void QuasiStatic::control_loop(){
    srl::Rate r{1./dt};
    while(true){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        //update the internal visualization
        if (sensor_type != CurvatureCalculator::SensorType::simulator) cc->get_curvature(state);
        
        stm->updateState(state);
        
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;

        J = stm->J[st_params.num_segments-1]; //tip jacobian
        //J_mid << stm->J[st_params.num_segments-2], stm->J[st_params.num_segments-1] ; //middle seg jacobian

        //this x is from qualisys directly
        x = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params.num_segments).translation()-cc->get_frame(0).translation());
        //this x is from forward kinematics, use when using bendlabs sensors

        dx = J*state.dq;
        
        ddx_des = ddx_ref + kp*(x_ref - x).normalized()*0.05;            //desired acceleration from PD controller
    //normed to always assume a distance of 5cm


        for (int i = 0; i < singularity(J); i++){               //reduce jacobian order if the arm is in a singularity
            J.block(0,(st_params.num_segments-1-i)*2,3,2) += 0.02*(i+1)*MatrixXd::Identity(3,2);
        }


        tau_ref = J.transpose()*ddx_des;
        VectorXd pxy = stm->A_pseudo.inverse()*tau_ref/10000;
        p = stm->pseudo2real(pxy + p_prev);
        p_prev += pxy;
        if (sensor_type != CurvatureCalculator::SensorType::simulator) {actuate(p);}
        else {
            assert(simulate(p));
        }
    }
}

int QuasiStatic::singularity(const MatrixXd &J) {
    int order = 0;
    std::vector<Eigen::Vector3d> plane_normals(st_params.num_segments);            //normals to planes create by jacobian
    for (int i = 0; i < st_params.num_segments; i++) {
        Vector3d j1 = J.col(2*i).normalized();   //Eigen hates fun so we have to do this
        Vector3d j2 = J.col(2*i+1).normalized();
        plane_normals[i] = j1.cross(j2);
    }

    for (int i = 0; i < st_params.num_segments - 1; i++) {                         //check for singularities
        for (int j = 0; j < st_params.num_segments - 1 - i; j++){
            if (abs(plane_normals[i].dot(plane_normals[i+j+1])) > 0.995) order+=1;  //if the planes are more or less the same, we are near a singularity
        }
    }
    return order;
}


double QuasiStatic::get_kd(){
    return this->kd;
}
double QuasiStatic::get_kp(){
    return this->kp;
}
void QuasiStatic::set_kd(double kd){
    this->kd = kd;
}
void QuasiStatic::set_kp(double kp){
    this->kp = kp;
}