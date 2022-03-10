#include "3d-soft-trunk/Controllers/OSC.h"

OSC::OSC(const SoftTrunkParameters st_params) : ControllerPCC::ControllerPCC(st_params){
    filename_ = "OSC_logger";

    potfields_.resize(st_params_.objects);
    for (int i = 0; i < st_params_.objects; i++) {
        potfields_[i].cutoff_distance_ = 0.5;
        potfields_[i].strength_ = 0.11;
        potfields_[i].radius_ = 0.045;
    }

    //set the gains
    kp_ = 170;
    kd_ = 5.5;


    //OSC needs a higher refresh rate than other controllers
    dt_ = 1./80;

    control_thread_ = std::thread(&OSC::control_loop, this);
}

void OSC::control_loop() {
    srl::Rate r{1./dt_};
    while(true){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        //update the internal visualization
        
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;

        dyn_ = mdl_->dyn_;
        J = dyn_.J[st_params_.num_segments-1+st_params_.prismatic];

        x_ = state_.tip_transforms[st_params_.num_segments+st_params_.prismatic].translation();
        assert(x_ == Vector3d::Zero()); //reminder to check for an easy way to grab x tip from qualisys
        //this x is from forward kinematics, use when using bendlabs sensors


        dx_ = J*state_.dq;
        
        ddx_des = ddx_ref_ + kp_*(x_ref_ - x_) + kd_*(dx_ref_ - dx_);            //desired acceleration from PD controller

        if ((x_ref_ - x_).norm() > 0.05) { //saturate the proportional gain
            ddx_des = ddx_ref_ + kp_*(x_ref_ - x_).normalized()*0.05 + kd_*(dx_ref_ - dx_);  
        }

        for (int i = 0; i < potfields_.size(); i++) {            //add the potential fields from objects to reference
            if (!freeze){
                potfields_[i].pos_ = state_.objects[i].translation();
            }
            ddx_des += potfields_[i].get_ddx(x_); 
        }

        for (int i = 0; i < singularity(J); i++){               //reduce jacobian order if the arm is in a singularity
            J.block(0,(st_params_.num_segments-1-i)*2,3,2) += 0.02*(i+1)*MatrixXd::Identity(3,2); //noise should fix it
        }
        

        B_op = (J*dyn_.B.inverse()*J.transpose()).inverse();
        J_inv = computePinv(J, 0.5e-1, 1.0e-1);
         
        f_ = B_op*ddx_des;
        
        f_(0) += loadAttached_;
        f_(2) += loadAttached_ + 0.24*gripperAttached_; //the gripper weights 0.24 Newton

        tau_null = -kd_*0.0001*state_.dq;
        
        for(int i = 0; i < st_params_.q_size; i++){     //for some reason tau is sometimes nan, catch that
            if(isnan(tau_null(i))) tau_null = VectorXd::Zero(2*st_params_.num_segments);
        }

        tau_ref = J.transpose()*f_ + (MatrixXd::Identity(st_params_.q_size, st_params_.q_size) - J.transpose()*J_inv.transpose())*tau_null;
        
        p_ = mdl_->pseudo2real(dyn_.A_pseudo.inverse()*tau_ref/100 + mdl_->gravity_compensate(state_));

        if (st_params_.sensors[0] != SensorType::simulator) {actuate(p_);}
        else {
            assert(simulate(p_));
        }
    }
}



PotentialField::PotentialField(){
    this->pos_ = Vector3d::Zero();
    this->strength_ = 0.005;
    this->cutoff_distance_ = 0.1;
    this->radius_ = 0.05;
}

PotentialField::PotentialField(Vector3d &pos, double s){
    this->pos_ = pos;
    this->strength_ = s;
    this->cutoff_distance_ = 0.1;
    this->radius_ = 0.05;
}

Vector3d PotentialField::get_ddx(Vector3d &pos) {
    Vector3d differential = pos - this->pos_;
    double distance = differential.norm() - radius_;
    if (distance < this->cutoff_distance_) {
        return strength_ * 1./distance * differential.normalized();
    }
    return Vector3d::Zero();
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

//return pseudo inverse computed in dampedPseudoInverse function 
MatrixXd OSC::computePinv(Eigen::MatrixXd j,double e,double lambda)
{

	Eigen::MatrixXd pJacobian(j.cols(), j.rows());
	dampedPseudoInverse(j,e,lambda, pJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
	return pJacobian;
}