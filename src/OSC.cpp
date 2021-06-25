#include "3d-soft-trunk/OSC.h"

OSC::OSC(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, bool simulation, int objects) : ControllerPCC::ControllerPCC(st_params, sensor_type, simulation, objects){
    filename = "OSC_logger";

    potfields.resize(objects);
    for (int i = 0; i < objects; i++) {
        potfields[i].set_cutoff(0.1);
        potfields[i].set_strength(0.005);
        potfields[i].set_radius(0.001);
    }

    J_mid = MatrixXd::Zero(3*st_params.num_segments, st_params.q_size);

    //set the gains
    kp = 35;
    kd = 5.3;
    ki = 0.;
    ki_gain = 1;

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
        if (!simulation) cc->get_curvature(state);
        
        stm->updateState(state);
        
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;

        J = stm->J[st_params.num_segments-1]; //tip jacobian
        J_mid << stm->J[st_params.num_segments-2], stm->J[st_params.num_segments-1] ; //middle seg jacobian

        //do controls
        x = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params.num_segments).translation()-cc->get_frame(0).translation());
        //x = stm->get_H_base().rotation()*stm->get_H(st_params.num_segments-1).translation();
        x_mid = stm->get_H_base().rotation()*stm->get_H(st_params.num_segments-2).translation();

        dx = J*state.dq;
        //fmt::print("dx = {}\n", dx.norm());


        ddx_ref = (kp + ki)*(x_ref - x) + kd*(dx_ref - dx);            //desired acceleration from PD controller
        ddx_null = VectorXd::Zero(3*st_params.num_segments);

        for (int i = 0; i < potfields.size(); i++) {            //add the potential fields from objects to reference
            potfields[i].set_pos(get_objects()[i]);
            //ddx_ref += potfields[i].get_ddx(x);
            //ddx_null.segment(0,3) += potfields[i].get_ddx(x_mid);
        }

        for (int i = 0; i < singularity(J); i++){               //reduce jacobian order if the arm is in a singularity
            J.block(0,(st_params.num_segments-1-i)*2,3,2) += 0.5*(i+1)*MatrixXd::Identity(3,2);
        }
        for (int j = 0; j < st_params.num_segments; j++){
            for (int i = 0; i < singularity(J_mid.block(3*j,0,3,st_params.q_size)); i++){               //reduce jacobian order if the arm is in a singularity
                J_mid.block(3*j,2*i,3,2) + 0.2*(i+1)*MatrixXd::Identity(3,2);
            }
        }

        B_op = (J*stm->B.inverse()*J.transpose()).inverse();
        J_inv = stm->B.inverse()*J.transpose()*B_op;

        B_op_null = (J_mid*stm->B.inverse()*J_mid.transpose()).inverse();
         
        f = B_op*ddx_ref;
        
        f_null = B_op_null*ddx_null;
        //if (gripperAttached) f(2) += 0.16; //the gripper weighs 24 grams -> 0.24 Newto

        tau_null = J_mid.transpose()*f_null;
        
        for(int i = 0; i < st_params.q_size; i++){     //for some reason tau is sometimes nan, catch that
            if(isnan(tau_null(i))) tau_null = VectorXd::Zero(2*st_params.num_segments);
        }

        tau_ref = J.transpose()*f + stm->D * state.dq + (MatrixXd::Identity(st_params.q_size, st_params.q_size) - J.transpose()*J_inv.transpose())*tau_null;
        
        p = stm->pseudo2real(stm->A_pseudo.inverse()*tau_ref/100) + stm->pseudo2real(gravity_compensate(state));

        if (!simulation) {actuate(p);}
        else {
            assert(simulate(p));
        }
    }
}

int OSC::singularity(const MatrixXd &J) {
    int order = 0;
    std::vector<Eigen::Vector3d> plane_normals(st_params.num_segments);            //normals to planes create by jacobian
    for (int i = 0; i < st_params.num_segments; i++) {
        Vector3d j1 = J.col(2*i).normalized();   //Eigen hates fun so we have to do this
        Vector3d j2 = J.col(2*i+1).normalized();
        plane_normals[i] = j1.cross(j2);
    }

    for (int i = 0; i < st_params.num_segments - 1; i++) {                         //check for singularities
        for (int j = 0; j < st_params.num_segments - 1 - i; j++){
            if (abs(plane_normals[i].dot(plane_normals[i+j+1])) > 0.99) order+=1;  //if the planes are more or less the same, we are near a singularity
        }
    }
    return order;
}

PotentialField::PotentialField(){
    this->pos = Vector3d::Zero();
    this->strength = 0.005;
    this->cutoff_distance = 0.1;
    this->radius = 0.05;
}

PotentialField::PotentialField(Vector3d &pos, double s){
    this->pos = pos;
    this->strength = s;
    this->cutoff_distance = 0.1;
    this->radius = 0.05;
}

Vector3d PotentialField::get_ddx(Vector3d &pos) {
    Vector3d differential = pos - this->pos;
    double distance = differential.norm() - radius;
    if (distance < this->cutoff_distance) {
        return strength * (1./distance - 1./cutoff_distance) * 1./distance * differential.normalized();
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

void PotentialField::set_radius(double r) {
    assert(r > 0.);
    this->radius = r;
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