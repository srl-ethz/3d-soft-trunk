#include "3d-soft-trunk/Adaptive.h"
using namespace std;

Adaptive::Adaptive(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects) : ControllerPCC::ControllerPCC(st_params, sensor_type, objects)
{

    filename = "ID_logger";

    Kp << 75.0, 75.0, 75.0; //control gains
    Kd << 0.1, 0.1, 0.1;       //control gains
    knd = 10.0;                //null space damping gain
    dt = 1. / 100;             //controller's rate

    eps = 0.1;     //for pinv of Jacobian
    lambda = 0.05; //for pinv of Jacobian

    gamma1 = 0.001;    //control gains
    gamma2 = 0.01; //control gains

    delta = 0.05; //boundary layer tickness

    rate = 0.0000001; //variation rate of estimates; may remove one zero
    // maybe use a diag matrix instead of double to decrease this rate for inertia params.
    // already included in Ka

    alpha = 0.75; //Finite time stability

    control_thread = std::thread(&Adaptive::control_loop, this);
    // initialize dynamic parameters
    a << 0.0038, 0.0022, 0.0015, 0.0018, 0.0263, 0.0153, 0.0125, 0.001, 0.001, 0.2500, 0.150;
}

void Adaptive::control_loop()
{

    SoftTrunkParameters st_params_l{};
    st_params_l.load_yaml("softtrunkparams_Lagrange.yaml");
    st_params_l.finalize(); // run sanity check and finalize parameters

    Lagrange lag(st_params_l);
    srl::Rate r{1. / dt};
    while (true)
    {
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);
        cc->get_curvature(state);
        stm->updateState(state);

        avoid_singularity(state);
        lag.update(state, state_ref);
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;
        x = lag.p;
        x_qualisys = stm->get_H_base().rotation() * cc->get_frame(0).rotation() * (cc->get_frame(st_params.num_segments).translation() - cc->get_frame(0).translation());
        //x = x_qualiszs;
        dx = lag.J * state.dq;
        //ddx_d = ddx_ref + Kp.asDiagonal() * (x_ref - x) + Kd.asDiagonal() * (dx_ref - dx);
        e = x_ref - x;
        eDot = dx_ref - dx;
        J_inv = computePinv(lag.J, eps, lambda);

        //state_ref.dq = J_inv * (dx_ref + 0.1*Kp.asDiagonal() * (x_ref - x));
        //state_ref.ddq = J_inv * (ddx_d - lag.JDot * state_ref.dq) + ((MatrixXd::Identity(state.q.size(), state.q.size()) - J_inv * lag.J)) * (-knd * state.dq);
        v = Kp.array()*e.array().abs().pow(alpha)*sat(e, 0).array();
        //state_ref.dq = J_inv * (dx_ref + 0.1*v);
        state_ref.dq = J_inv * (dx_ref + 0.3*v + 0.3*1*Kp.asDiagonal() * (x_ref - x));
        vDot = alpha*Kd.array()*e.array().abs().pow(alpha-1)*eDot.array();
        state_ref.ddq = J_inv * (ddx_ref + Kp.asDiagonal()*e + 1*Kd.asDiagonal() * eDot + vDot -lag.JDot * state_ref.dq);
        lag.update(state, state_ref); //update again for state_ref to get Y

        s = state.dq - state_ref.dq;     //sliding manifold
        s_d = s - delta * sat(s, delta); //manifold with boundary layer

        aDot = -1 * Ka.asDiagonal() * lag.Y.transpose() * s_d; //Adaptation law
        a = a + rate * dt * aDot;                         //integrate the estimated dynamic parameters parameters
        avoid_drifting();                                 // keep the dynamic parameters within range

        //cout << "\na \n " << a << "\n\n";
        Ainv = computePinv(lag.A, eps, lambda);                       // compute pesudoinverse of mapping matrix
        tau = Ainv * lag.Y * a - gamma1 * s - gamma2 * sat(s, delta); // compute the desired toque in xy
        p = stm->pseudo2real(stm->A_pseudo.inverse() * tau / 100);    // compute the desired pressure
        actuate(p);                                                   // control the valves
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

void Adaptive::avoid_drifting() //keeps the dynamic parameters within the range
{
    for (int i = 0; i < Ka.size(); i++)
    {
        if (a(i) < a_min(i) || a(i) > a_max(i))
        {
            Ka(i) = 0;
            a(i) = std::min(a_max(i), std::max(a(i), a_min(i)));
        }
        else
            Ka(i) = Ka_(i);
    }
}

/*
VectorXd Adaptive::sat(VectorXd s, double delta)
{
    VectorXd Delta = VectorXd::Ones(s.size())*delta;
    return s.array()/(s.array().abs()+Delta.array());
}
*/
VectorXd Adaptive::sat(VectorXd x, double delta)
{
    VectorXd s = VectorXd::Zero(x.size());
    for (int i = 0; i < x.size(); i++)
        s(i) = (std::abs(x(i)) >= delta) ? sign(x(i)) : x(i) / delta;
    return s;
}

double Adaptive::sign(double val)
{
    if (val == 0)
        return 0.0;
    else if (val > 0)
        return 1.0;
    else
        return -1.0;
}

void Adaptive::increase_kd()
{
    this->Kd = 1.1 * this->Kd;
    fmt::print("kd = {}\n", Kd(0));      
}
void Adaptive::increase_kp()
{
    this->Kp = 1.1 * this->Kp;
    fmt::print("kp = {}\n", Kp(0));           
}

void Adaptive::decrease_kd()
{
    this->Kd = 0.9 * this->Kd;
    fmt::print("kd = {}\n", Kd(0));
}

void Adaptive::decrease_kp()
{
    this->Kp = 0.9 * this->Kp;
    fmt::print("kp = {}\n", Kp(0));
}

void Adaptive::increase_gamma1()
{
    this->gamma1 = 1.1 * this->gamma1;
    fmt::print("gamma1 = {}\n", gamma1);
}

void Adaptive::decrease_gamma1()
{
    this->gamma1 = 0.9 * this->gamma1;
    fmt::print("gamma1 = {}\n", gamma1);
}

void Adaptive::increase_gamma2()
{
    this->gamma2 = 1.1 * this->gamma2;
    fmt::print("gamma2 = {}\n", gamma2);
}

void Adaptive::decrease_gamma2()
{
    this->gamma2 = 0.9 * this->gamma2;
    fmt::print("gamma2 = {}\n", gamma2);
}

void Adaptive::increase_delta()
{
    this->delta = 1.1 * this->delta;
    fmt::print("delta = {}\n", delta);
}

void Adaptive::decrease_delta()
{
    this->delta = 0.9 * this->delta;
    fmt::print("delta = {}\n", delta);
}

void Adaptive::increase_rate()
{
    this->rate = 1.1 * this->rate;
    fmt::print("rate = {}\n", rate);
}

void Adaptive::decrease_rate()
{
    this->rate = 0.9 * this->rate;
    fmt::print("rate = {}\n", rate);
}

void Adaptive::increase_eps()
{
    this->eps = 1.1 * this->eps;
    fmt::print("eps = {}\n", eps);
}

void Adaptive::decrease_eps()
{
    this->eps = 0.9 * this->eps;
    fmt::print("eps = {}\n", eps);
}

void Adaptive::increase_lambda()
{
    this->lambda = 1.1 * this->lambda;
    fmt::print("lambda = {}\n", lambda);
}

void Adaptive::decrease_lambda()
{
    this->lambda = 0.9 * this->lambda;
    fmt::print("lambda = {}\n", lambda);
}

void Adaptive::increase_stiffness(int seg){
    this->a[9+seg] = this->a[9+seg] * 1.1;
    fmt::print("stiffness{}: {}\n", seg, this->a[9+seg]);
}

void Adaptive::decrease_stiffness(int seg){
    this->a[9+seg] = this->a[9+seg] * 0.9;
    fmt::print("stiffness{}: {}\n", seg, this->a[9+seg]);
}

void Adaptive::increase_damping(){
    this->a[7] = this->a[7] * 1.1;
    this->a[8] = this->a[8] * 1.1;
    fmt::print("damping: {}\n", this->a[8]);
}

void Adaptive::decrease_damping(){
    this->a[7] = this->a[7] * 0.9;
    this->a[8] = this->a[8] * 0.9;
    fmt::print("damping: {}\n", this->a[8]);
}

void Adaptive::change_ref()
{
    x_ref << 0.0,0.14,-0.23;
    fmt::print("position_changed = {}\n", x_ref);
}

void Adaptive::show_x()
{
    fmt::print("qlysis_position = {}\n", x_qualisys);
}


