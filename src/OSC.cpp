#include "3d-soft-trunk/OSC.h"

OSC::OSC(CurvatureCalculator::SensorType sensor_type, bool simulation, int objects) : ControllerPCC::ControllerPCC(sensor_type, simulation, objects){
    filename = "OSC_logger";

    potfields.resize(objects);
    for(int i = 0; i < potfields.size(); i++){
        potfields[i].set_strength(0.005);
        potfields[i].set_cutoff(0.1);
    }

    //set the gains
    kp = 35;
    kd = 5.5;

    //OSC needs a higher refresh rate than other controllers
    dt = 1./50;

    control_thread = std::thread(&OSC::control_loop, this);
}

void OSC::control_loop() {
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
        J_mid = stm->J[st_params::num_segments-2]; //middle seg jacobian

        //do controls
        x = stm->get_H_base().rotation()*stm->get_H(st_params::num_segments-1).translation();
        x_mid = stm->get_H_base().rotation()*stm->get_H(st_params::num_segments-2).translation();

        dx = J*state.dq;
        ddx_ref = kp*(x_ref - x) + kd*(dx_ref - dx);            //desired acceleration from PD controller
        ddx_null = Vector3d::Zero();

        for (int i = 0; i < potfields.size(); i++) {            //add the potential fields from objects to reference
            potfields[i].set_pos(get_objects()[i]);
            ddx_ref += potfields[i].get_ddx(x);
            ddx_null += potfields[i].get_ddx(x_mid);
        }

        for (int i = 0; i < singularity(J); i++){               //reduce jacobian order if the arm is in a singularity
            J.block(0,2*i,3,2) + 0.1*(i+1)*MatrixXd::Identity(3,2);//+ (i+1)*0.01*MatrixXd::Identity(3,2);
        }
        for (int i = 0; i < singularity(J_mid); i++){               //reduce jacobian order if the arm is in a singularity
            J_mid.block(0,2*i,3,2) + 0.1*(i+1)*MatrixXd::Identity(3,2);//+ (i+1)*0.01*MatrixXd::Identity(3,2);
        }


        B_op = (J*stm->B.inverse()*J.transpose()).inverse();
        J_inv = stm->B.inverse()*J.transpose()*B_op;

        B_op_null = (J_mid*stm->B.inverse()*J_mid.transpose()).inverse();
         
        f = B_op*ddx_ref;
        
        f_null = B_op_null*ddx_null;
        if (gripperAttached) f(2) += 0.24; //the gripper weighs 24 grams -> 0.24 Newto

        tau_null = VectorXd::Zero(4);//Jt.transpose()*f_null*0;
        tau_ref = /*J.transpose()*f*/J_mid.transpose()*f_null + stm->D * state.dq + (MatrixXd::Identity(st_params::q_size, st_params::q_size) - J.transpose()*J_inv.transpose())*tau_null;
        
        p = stm->pseudo2real(stm->A_pseudo.inverse()*tau_ref/100) + stm->pseudo2real(gravity_compensate(state));

        if (!simulation) {actuate(p);}
        else {simulate(p);}
        
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
            if (abs(plane_normals[i].dot(plane_normals[i+j+1])) > 0.99) order+=1;  //if the planes are more or less the same, we are near a singularity
        }
    }
    return order;
}

PotentialField::PotentialField(){
    this->pos = Vector3d::Zero();
    this->strength = 0.005;
    this->cutoff_distance = 1.5;
}

PotentialField::PotentialField(Vector3d &pos, double s){
    this->pos = pos;
    this->strength = s;
    this->cutoff_distance = 1.5;
}

Vector3d PotentialField::get_ddx(Vector3d &pos) {
    Vector3d differential = pos - this->pos;
    if (differential.norm() < this->cutoff_distance) {
        return strength * (1./differential.norm() - 1./cutoff_distance) * 1./(differential.norm()) * differential.normalized();
    }
    return Vector3d::Zero();
}

void PotentialField::set_pos(Vector3d &pos) {
    this->pos = pos;
}
void PotentialField::set_strength(double s){
    this->strength = s;
}
void PotentialField::set_cutoff(double c){
    this->cutoff_distance = c;
}
Vector3d PotentialField::get_pos(){
    return this->pos;
}
double PotentialField::get_strength(){
    return this->strength;
}
double PotentialField::get_cutoff(){
    return this->cutoff_distance;
}
double OSC::get_kd(){
    return this->kd;
}
double OSC::get_kp(){
    return this->kp;
}
void OSC::set_kd(double kd){
    this->kd = kd;
}
void OSC::set_kp(double kp){
    this->kp = kp;
}