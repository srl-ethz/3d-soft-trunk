#include "3d-soft-trunk/Adaptive.h"
using namespace std;

Adaptive::Adaptive(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects) : ControllerPCC::ControllerPCC(st_params, sensor_type, objects)
{
    
    filename = "ID_logger";
    Ka << 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.;
    Kp << 20., 20., 20.;
    Kd << 2, 2, 2;

    dt = 1. / 50;
    eps = 1e-5;
    lambda = 0.5e-5;
    control_thread = std::thread(&Adaptive::control_loop, this);

    //a << 0.0043, 0.0025, 0.0016, 0.0020, 0.0279, 0.0163, 0.0131, 0.0100, 0.0100, 0.1500, 0.0700;

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
        //std::cout << "\n q: \n" << state.q << "\n\n";
        avoid_singularity(state);
        lag.update(state, state_ref);

        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;
        // Todo: check x with x_tip
        //std::cout << "ref" << x_ref << "\n";
        x = lag.p;
        Vector3d x_qualiszs = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params.num_segments).translation()-cc->get_frame(0).translation());
        /*std::cout << "x_qualisys \n" << x_qualiszs << "\n\n";
        std::cout << "x_kinematic \n" << x << "\n\n";*/
        dx = lag.J * state.dq;
        //std::cout << "dx \n" << dx << "\n\n";
        ddx_d = ddx_ref + Kp.asDiagonal() * (x_ref - x) + Kd.asDiagonal() * (dx_ref - dx);
        J_inv = computePinv(lag.J, eps, lambda);
        state_ref.dq = J_inv * (dx_ref + Kp.asDiagonal() * (x_ref - x));
        //state_ref.dq = state.dq; //may also try this for regulation
        state_ref.ddq = J_inv * (ddx_d - lag.JDot * state.dq);//  + ((MatrixXd::Identity(st_params.q_size, st_params.q_size) - J_inv * lag.J)) * (-0.1 * state.dq);
        //state_ref.ddq = J_inv*(ddx_d - lag.JDot*state.dq) + ((MatrixXd::Identity(st_params.q_size, st_params.q_size) - J_inv*lag.J))*(-0.1*state.dq);
       
        lag.update(state, state_ref); //update again for state_ref to get Y
        
        //aDot = Ka.asDiagonal() * lag.Y.transpose() * (state_ref.dq - state.dq);
        //a += 0.001* dt * aDot;
        //cout << "\na \n " << a << "\n\n";
        //tau = lag.A.inverse() * lag.Y * a;
        MatrixXd Ainv;
        //Ainv = lag.A.inverse();
        Ainv = computePinv(lag.A, eps, lambda);
        //cout << "\n map \n" << Ainv << "\n";
        //tau = Ainv * lag.Y * a;
        //VectorXd dumm = lag.k * state.q + lag.d*state.dq;
        //cout << dumm;
        tau = Ainv * (lag.M*state_ref.ddq + lag.Cdq + lag.g + lag.k  + lag.d);
        /*cout << "\n tau: \n" << tau << "\n\n";
        cout << "\n pxy \n " << stm->A_pseudo.inverse() * tau / 100 << "\n\n";*/
        p = stm->pseudo2real(stm->A_pseudo.inverse() * tau / 100);
        //cout << "\n pressure_control \n " << p << "\n\n";
        /*
        p(0) = 0;
        p(1) = 400;
        p(2) = 400;
        p(3) = 0;
        p(4) = 400;
        p(5) = 400;
        cout << "\n pressure_feedforward \n " << p << "\n\n";
        */
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
    double eps_custom = 0.0001;
    for (int idx = 0; idx < state.q.size(); idx++)
    {
        r = std::fmod(std::abs(state.q[idx]), 6.2831853071795862); //remainder of division by 2*pi
        state.q[idx] = (r < eps_custom) ? eps_custom : state.q[idx];
    }
}

void Adaptive::increase_kd(){
    this->Kd = 1.1*this->Kd;
    fmt::print("kd = {}\n", Kd(0));
}
void Adaptive::increase_kp(){
    this->Kp = 1.1*this->Kp;
    fmt::print("kp = {}\n", Kp(0));
}

void Adaptive::decrease_kd(){
    this->Kd = 0.9*this->Kd;
    fmt::print("kd = {}\n", Kd(0));
}

void Adaptive::decrease_kp(){
    this->Kp = 0.9*this->Kp;
    fmt::print("kp = {}\n", Kp(0));
}
