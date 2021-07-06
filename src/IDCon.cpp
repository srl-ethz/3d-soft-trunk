// Inverse Dynamics approach With Singularity Avoidance 
// Amirhossein Kazemipour

#include "3d-soft-trunk/IDCon.h"

IDCon::IDCon(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects) : ControllerPCC::ControllerPCC(st_params, sensor_type, objects){
    filename = "ID_logger";
    J_prev = MatrixXd::Zero(3, st_params.q_size);
    kp = 70;
    kd = 5.5;
    dt = 1./50;
    control_thread = std::thread(&IDCon::control_loop, this);
    eps = 1e-1;
	lambda = 0.5e-1;
}
//
//
void IDCon::control_loop(){
    srl::Rate r{1./dt};
    while(true){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        //update the internal visualization
        if (!(sensor_type == CurvatureCalculator::SensorType::simulator)) cc->get_curvature(state);
        stm->updateState(state);
        
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;

        J = stm->J[st_params.num_segments-1]; //tip jacobian
        dJ = (J - J_prev)/dt;

        J_prev = J;
        //do controls
        x = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params.num_segments).translation()-cc->get_frame(0).translation());
        //x = stm->get_H_base().rotation()*stm->get_H(st_params::num_segments-1).translation();

        dx = J*state.dq;
        ddx_d = ddx_ref + kp*(x_ref - x) + kd*(dx_ref - dx); 
        //J_inv = J.transpose()*(J*J.transpose()).inverse();
        J_inv = computePinv(J, eps, lambda);
        state_ref.ddq = J_inv*(ddx_d - dJ*state.dq) + ((MatrixXd::Identity(st_params.q_size, st_params.q_size) - J_inv*J))*(-kd*state.dq);

        tau_ref = stm->B*state_ref.ddq + stm->c + stm->g + stm->K * state.q + stm->D*state.dq;
        
        p = stm->pseudo2real(stm->A_pseudo.inverse()*tau_ref/100);

        if (!(sensor_type == CurvatureCalculator::SensorType::simulator)) {actuate(p);}
        else {
            assert(simulate(p));
        }
    }

}

double IDCon::get_kd(){
    return this->kd;
}
double IDCon::get_kp(){
    return this->kp;
}
void IDCon::set_kd(double kd){
    this->kd = kd;
}
void IDCon::set_kp(double kp){
    this->kp = kp;
}

//compute damped pesudo inverse
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
Eigen::MatrixXd IDCon::computePinv(Eigen::MatrixXd j,double e,double lambda)
{

	Eigen::MatrixXd pJacobian(j.cols(), j.rows());
	dampedPseudoInverse(j,e,lambda, pJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
	return pJacobian;
}