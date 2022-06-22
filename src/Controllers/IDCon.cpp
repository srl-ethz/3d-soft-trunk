// Inverse Dynamics approach With Singularity Avoidance 
// Amirhossein Kazemipour

#include "3d-soft-trunk/Controllers/IDCon.h"

IDCon::IDCon(const SoftTrunkParameters st_params) : ControllerPCC::ControllerPCC(st_params){
    filename_ = "ID_logger";
    J_prev = MatrixXd::Zero(3, st_params.q_size);
    kp = 70;
    kd = 5.5;
    dt_ = 1./50;
    control_thread_ = std::thread(&IDCon::control_loop, this);
    eps = 1e-1;
	lambda = 0.5e-1;
    fmt::print("IDCon initialized.\n");
}
//
//
void IDCon::control_loop(){
    srl::Rate r{1./dt_};
    while(true){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);
        
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;

        J = dyn_.J[st_params_.num_segments-1+st_params_.prismatic]; //tip jacobian
        dJ = (J - J_prev)/dt_;

        J_prev = J; //for JDot
        
        x_ = state_.tip_transforms[st_params_.num_segments+st_params_.prismatic].translation();
        dx_ = J*state_.dq;
        ddx_d = ddx_ref + kp*(x_ref_ - x_) + kd*(dx_ref_ - dx_); 

        //J_inv = J.transpose()*(J*J.transpose()).inverse();
        J_inv = computePinv(J, eps, lambda); //use a damped pseudoinverse, since normal Moore-Penrose was wobbly

        //inverse dynamics, for detailed explanation check out "Operational Space Control: Empirical and Theoretical Comparison"
        state_ref_.ddq = J_inv*(ddx_d - dJ*state_.dq) + ((MatrixXd::Identity(st_params_.q_size, st_params_.q_size) - J_inv*J))*(-kd*state_.dq);

        tau_ref = dyn_.B*state_ref_.ddq + gravity_compensate(state_);
        
        p_ = mdl_->pseudo2real(dyn_.A_pseudo.inverse()*tau_ref/100);

        //actuate(p_);
    }

}

//compute damped pesudo inverse with a variable damping
//Deo, A. S., & Walker, I. D. (1995). Overview of damped least-squares methods for inverse kinematics of robot manipulators. Journal of Intelligent and Robotic Systems, 14(1), 43-68.
//Flacco, Fabrizio, and Alessandro De Luca. "A reverse priority approach to multi-task control of redundant robots." 2014 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2014.
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

void IDCon::circle(int pressure, double period){
    toggle_log();
    srl::Rate r{1./dt_};
    double t = 0;

    double c = 2 * 3.1415 / period;

    VectorXd p = VectorXd::Zero(4);

    p << pressure, 0, pressure, 0;
    srl::sleep(5);
    while (t < 3*period){
        for (int i = 0; i < st_params_.num_segments; i++){
            p(2*i) = cos(c*t) * pressure;
            p(2*i+1) = sin(c*t) * pressure;
        }
        actuate(mdl_->pseudo2real(p));
        r.sleep();
    }
}