#include "3d-soft-trunk/OSC.h"

OSC::OSC(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects) : ControllerPCC::ControllerPCC(st_params, sensor_type, objects){
    filename = "OSC_logger";

    potfields.resize(objects);
    for (int i = 0; i < objects; i++) {
        potfields[i].set_cutoff(0.1);
        potfields[i].set_strength(0.005);
        potfields[i].set_radius(0.001);
    }

    J_mid = MatrixXd::Zero(3*st_params.num_segments, st_params.q_size);

    //set the gains
    kp = 60;
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
        if (sensor_type != CurvatureCalculator::SensorType::simulator) cc->get_curvature(state);
        
        stm->updateState(state);
        
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;

        J = stm->J[st_params.num_segments-1]; //tip jacobian
        //J_mid << stm->J[st_params.num_segments-2], stm->J[st_params.num_segments-1] ; //middle seg jacobian

        //this x is from qualisys directly
        x = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params.num_segments).translation()-cc->get_frame(0).translation());
        //this x is from forward kinematics, use when using bendlabs sensors
        //x = stm->get_H_base().rotation()*stm->get_H(st_params.num_segments-1).translation();
        
        
        //x_mid = stm->get_H_base().rotation()*stm->get_H(st_params.num_segments-2).translation();

        dx = J*state.dq;
        
        ddx_des = ddx_ref + kp*(x_ref - x) + kd*(dx_ref - dx);            //desired acceleration from PD controller

        double distance = (x - x_ref).norm();
        //if (distance > 0.15) ddx_des = ddx_ref + kp*(x_ref - x).normalized()*0.15 + kd*(dx_ref - dx);

        ddx_null = VectorXd::Zero(3*st_params.num_segments);

        for (int i = 0; i < potfields.size(); i++) {            //add the potential fields from objects to reference
            potfields[i].set_pos(get_objects()[i]);
            //ddx_des += potfields[i].get_ddx(x);
            //ddx_null.segment(0,3) += potfields[i].get_ddx(x_mid);
        }

        for (int i = 0; i < singularity(J); i++){               //reduce jacobian order if the arm is in a singularity
            J.block(0,(st_params.num_segments-1-i)*2,3,2) += 0.5*(i+1)*MatrixXd::Identity(3,2);
        }
        
        /*for (int j = 0; j < st_params.num_segments; j++){
            for (int i = 0; i < singularity(J_mid.block(3*j,0,3,st_params.q_size)); i++){               //reduce jacobian order if the arm is in a singularity
                J_mid.block(3*j,2*i,3,2) + 0.2*(i+1)*MatrixXd::Identity(3,2);
            }
        }*/

        B_op = (J*stm->B.inverse()*J.transpose()).inverse();
        //J_inv = stm->B.inverse()*J.transpose()*B_op;
        J_inv = computePinv(J, 0.5e-1, 1.0e-1);

        //B_op_null = (J_mid*stm->B.inverse()*J_mid.transpose()).inverse();
         
        f = B_op*ddx_des;
        
        //f_null = B_op_null*ddx_null;

        f(2) += loadAttached + 0.24*gripperAttached; //the gripper weights 0.24 Newton

        tau_null = -kd * state.dq;//J_mid.transpose()*f_null;
        
        for(int i = 0; i < st_params.q_size; i++){     //for some reason tau is sometimes nan, catch that
            if(isnan(tau_null(i))) tau_null = VectorXd::Zero(2*st_params.num_segments);
        }

        tau_ref = J.transpose()*f + stm->D * state.dq + (MatrixXd::Identity(st_params.q_size, st_params.q_size) - J.transpose()*J_inv.transpose())*tau_null;
        
        p = stm->pseudo2real(stm->A_pseudo.inverse()*tau_ref/100 + gravity_compensate(state));

        if (sensor_type != CurvatureCalculator::SensorType::simulator) {actuate(p);}
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
            if (abs(plane_normals[i].dot(plane_normals[i+j+1])) > 0.995) order+=1;  //if the planes are more or less the same, we are near a singularity
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

//courtesy of amir
template <typename Derived1, typename Derived2>
void dampedPseudoInverse(const Eigen::MatrixBase<Derived1>& A, double e, double dampingFactor, Eigen::MatrixBase<Derived2>& Apinv, unsigned int computationOptions)
{
	int m = A.rows(), n = A.cols(), k = (m < n) ? m : n;
	JacobiSVD<typename MatrixBase<Derived1>::PlainObject> svd = A.jacobiSvd(computationOptions);
	const typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType& singularValues = svd.singularValues();
	MatrixXd sigmaDamped = MatrixXd::Zero(k, k);
	double damp = dampingFactor * dampingFactor;

	for (int idx = 0; idx < k; idx++)
	{
		if (singularValues(idx) >= e)damp = 0;
		else damp = (1 - ((singularValues(idx) / e)*(singularValues(idx) / e)))*dampingFactor * dampingFactor;
		sigmaDamped(idx, idx) = singularValues(idx) / ((singularValues(idx) * singularValues(idx)) + damp);
	}
	Apinv = svd.matrixV() * sigmaDamped * svd.matrixU().transpose(); // damped pseudoinverse
}

//return pesudo inverse computed in dampedPseudoInverse function 
MatrixXd OSC::computePinv(Eigen::MatrixXd j,double e,double lambda)
{

	Eigen::MatrixXd pJacobian(j.cols(), j.rows());
	dampedPseudoInverse(j,e,lambda, pJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
	return pJacobian;
}