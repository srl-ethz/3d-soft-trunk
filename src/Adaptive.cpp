#include "3d-soft-trunk/Adaptive.h"
using namespace std;

Adaptive::Adaptive(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects) : ControllerPCC::ControllerPCC(st_params, sensor_type, objects)
{
    
    filename = "ID_logger";

    //Ka << 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.;
    //Ka_ << 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.;

    Kp << 165.0, 165.0, 165.0;
    Kd << 0.1, 0.1, 0.1;

    dt = 1. / 100;

    eps = 0.1;
    lambda = 0.05;

    gamma1 = 0.01;
    gamma2 = 2*0.5;

    control_thread = std::thread(&Adaptive::control_loop, this);
    // initialize dynamic parameters
    a << 0.0038  ,  0.0022  ,  0.0015 ,   0.0018 ,   0.0263 ,   0.0153  ,  0.0125 ,   0.0100  ,  0.0100   , 0.1400 ,   0.0700;
    fmt::print("Adaptive initialized.\n");
}

void Adaptive::control_loop()
{
    
    SoftTrunkParameters st_params_l{};
    st_params_l.load_yaml("softtrunkparams_Lagrange.yaml");
    st_params_l.finalize();  // run sanity check and finalize parameters

    Lagrange lag(st_params_l);
    srl::Rate r{1. / dt};
    while (true)
    {
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);
        //update the internal visualization
        cc->get_curvature(state);
        stm->updateState(state);
        avoid_singularity(state);
        lag.update(state, state_ref);
        //if (!is_initial_ref_received) //only control after receiving a reference position
        //    continue;
        x = lag.p;
        x_qualiszs = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params.num_segments).translation()-cc->get_frame(0).translation());

        //std::cout << "x_qualisys \n" << x_qualiszs << "\n\n";
        dx = lag.J * state.dq;
        ddx_d = ddx_ref + Kp.asDiagonal() * (x_ref - x) + Kd.asDiagonal() * (dx_ref - dx);
        J_inv = computePinv(lag.J, eps, lambda);
        state_ref.dq = J_inv * (dx_ref + Kp.asDiagonal() * (x_ref - x));
        state_ref.ddq = J_inv * (ddx_d - lag.JDot * state.dq)  + ((MatrixXd::Identity(st_params.q_size, st_params.q_size) - J_inv * lag.J)) * (-10 * state.dq);
       
        lag.update(state, state_ref); //update again for state_ref to get Y
        s = state_ref.dq - state.dq;
        
        aDot = Ka.asDiagonal() * lag.Y.transpose() * s;
        //a = a + 0.000001* dt * aDot;
        a = a + 0.0001* dt * aDot;

        for (int i=0; i< Ka.size(); i++)
        {
            if (a(i) < a_min(i) || a(i) > a_max(i)){
                Ka(i) = 0;
                a(i) = std::min(a_max(i),std::max(a(i),a_min(i)));
            }
            else
                Ka(i) = Ka_(i);
        }

        cout << "\na \n " << a << "\n\n";
        Ainv = computePinv(lag.A, eps, lambda);
        tau = Ainv * lag.Y * a + gamma1 * s + gamma2 * sat(s,0.1);

        p = stm->pseudo2real(stm->A_pseudo.inverse() * tau / 100);
        actuate(p);
    }
}

//compute damped pesudo inverse with a variable damping
//Deo, A. S., & Walker, I. D. (1995). Overview of damped least-squares methods for inverse kinematics of robot manipulators. Journal of Intelligent and Robotic Systems, 14(1), 43-68.
//Flacco, Fabrizio, and Alessandro De Luca. "A reverse priority approach to multi-task control of redundant robots." 2014 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2014.
template <typename Derived1, typename Derived2>
void dampedPseudoInverse(const Eigen::MatrixBase<Derived1> &A, double e, double dampingFactor, Eigen::MatrixBase<Derived2> &Apinv, unsigned int computationOptions)
{
    int m = A.rows(), n = A.cols(), k = (m < n) ? m : n;
    JacobiSVD<typename MatrixBase<Derived1>::PlainObject> svd = A.jacobiSvd(computationOptions);
    const typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType &singularValues = svd.singularValues();
    MatrixXd sigmaDamped = MatrixXd::Zero(k, k);
    double damp = dampingFactor * dampingFactor;

    for (int idx = 0; idx < k; idx++)
    {
        if (singularValues(idx) >= e)
            damp = 0;
        else
            damp = (1 - ((singularValues(idx) / e) * (singularValues(idx) / e))) * dampingFactor * dampingFactor;
        sigmaDamped(idx, idx) = singularValues(idx) / ((singularValues(idx) * singularValues(idx)) + damp);
    }
    Apinv = svd.matrixV() * sigmaDamped * svd.matrixU().transpose(); // damped pseudoinverse
}

//return pesudo inverse computed in dampedPseudoInverse function
Eigen::MatrixXd Adaptive::computePinv(Eigen::MatrixXd j, double e, double lambda)
{

    Eigen::MatrixXd pJacobian(j.cols(), j.rows());
    dampedPseudoInverse(j, e, lambda, pJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return pJacobian;
}

void Adaptive::avoid_singularity(srl::State &state)
{
    double r;
    double eps_custom = 0.05;
    for (int idx = 0; idx < state.q.size(); idx++)
    {
        r = std::fmod(std::abs(state.q[idx]), 6.2831853071795862); //remainder of division by 2*pi
        state.q[idx] = (r < eps_custom) ? eps_custom : state.q[idx];
    }
}

VectorXd Adaptive::sat(VectorXd s, double delta)
{
    VectorXd Delta = VectorXd::Ones(s.size())*delta;
    return s.array()/(s.array().abs()+Delta.array());
}

void Adaptive::increase_kd(){
    this->Kd = 1.1*this->Kd;
    fmt::print("kd = {}\n", Kd(0));
    std::cout << "x_qualisys \n" << x_qualiszs << "\n\n";
}
void Adaptive::increase_kp(){
    this->Kp = 1.1*this->Kp;
    fmt::print("kp = {}\n", Kp(0));
    std::cout << "x_qualisys \n" << x_qualiszs << "\n\n";
}

void Adaptive::decrease_kd(){
    this->Kd = 0.9*this->Kd;
    fmt::print("kd = {}\n", Kd(0));
    std::cout << "x_qualisys \n" << x_qualiszs << "\n\n";
}

void Adaptive::decrease_kp(){
    this->Kp = 0.9*this->Kp;
    fmt::print("kp = {}\n", Kp(0));
    std::cout << "x_qualisys \n" << x_qualiszs << "\n\n";
}