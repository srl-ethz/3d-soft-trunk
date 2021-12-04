#include "3d-soft-trunk/Adaptive.h"
#include <chrono>
using namespace std;

Adaptive::Adaptive(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects) : ControllerPCC::ControllerPCC(st_params, sensor_type, objects)
{

    filename = "Adaptive_logger";

    Kp = 120 * VectorXd::Ones(3);
    Kp << 150, 140, 170;
    Kd = 0.1 * VectorXd::Ones(3); //control gains
    knd = 10.0;                     //null space damping gain
    dt = 1. / 150;                  //controller's rate

    // eps = 0.5;     //for pinv of Jacobian star
    eps = 0.1; //circle
    lambda = 0.05; //for pinv of Jacobian

    gamma = 0.001;                //control gains
    b = 0.001 * VectorXd::Ones(4); //control gains

    delta = 0.05; //boundary layer tickness

    rate1 = 0; //variation rate of estimates; may remove one zero

    rate2 = 0; //variation rate of estimates; may remove one zero

    alpha = 0.75; //Finite time stability

    Ka(7) = 0.001;
    Ka(8) = 0.001;
    Ka(9) = 80;
    Ka(10) = 80; //2 for star
  
    eps_custom = 0.05; // for singularity avoidance
    
    // initialize dynamic parameters
    //a << 0.0038, 0.0022, 0.0015, 0.0018, 0.0263, 0.0153, 0.0125, 0.001, 0.001, 0.2, 0.07;
    //a << 0.0046, 0.0028, 0.0016, 0.0021, 0.0288, 0.0178, 0.0133, 0.006, 0.006, 0.30, 0.15;
    //a << 0.003528, 0.0031948, 0.002971, 0.003081, 0.0252, 0.02282, 0.022, 0.002, 0.002, 0.3, 0.1;
    zz = 1;
    double m[2] = {0.180,0.164};
    double L[2] = {0.151455,0.124099};
    double d_vect[2] = {0.001,0.001};
    //double k_vect[2] = {0.25,0.2};
    double k_vect[2] = {0.25,0.1};

    //initialize dynamic parameters
    a(0) = m[0]*L[0]*L[0];
    a(1) = m[1]*L[0]*L[0];
    a(2) = m[1]*L[1]*L[1];
    a(3) = m[1]*L[0]*L[1];
    a(4) = m[0]*L[0];
    a(5) = m[1]*L[0];
    a(6) = m[1]*L[1];
    a(7) = d_vect[0];
    a(8) = d_vect[1];
    a(9) = k_vect[0];
    a(10) = k_vect[1];

    
   target_points.resize(6);
   //coordinates_S();
    // this generates a star
    
   for(int i = 0; i < 6; i++){
       target_points[i] << 0.14*cos(144.*i*PI/180. + 1*PI/2), 0.14*sin(144.*i*PI/180. + 1*PI/2), -0.245;
       fmt::print("point {}: {}\n",i,target_points[i].transpose());
   }
    

   control_thread = std::thread(&Adaptive::control_loop, this);
}

void Adaptive::control_loop()
{

    SoftTrunkParameters st_params_l{};
    st_params_l.load_yaml("softtrunkparams_Lagrange.yaml");
    st_params_l.finalize(); // run sanity check and finalize parameters

    Lagrange lag(st_params_l);
    srl::Rate r{1. / dt};
    stm->updateState(state);
    while (true)
    {
        r.sleep();

        std::lock_guard<std::mutex> lock(mtx);
        //auto start_tot = std::chrono::system_clock::now();
        cc->get_curvature(state);
        cc->get_tip_posision(position);
        avoid_singularity(state);
       // auto start_mod = std::chrono::system_clock::now();
        lag.update(state, state_ref);
        //auto end_mod = std::chrono::system_clock::now();
        
       // if (!is_initial_ref_received) //only control after receiving a reference position
        //    continue;

        x = lag.p;
        
        //fmt::print("FK_position = {}\n", x);
        //fmt::print("FK_abs = {}\n", position);
        //fmt::print("des_pos = {}\n", x_ref);

        dx = lag.J * state.dq;
        //ddx_d = ddx_ref + Kp.asDiagonal() * (x_ref - x) + Kd.asDiagonal() * (dx_ref - dx);

        s_trapezoidal_speed(t_internal, &sigma, &dsigma, &ddsigma, &T);
        //fmt::print("pass1\n");
        Task_Circle_r2r(sigma, dsigma, ddsigma);
        //Task_Linear_r2r(sigma, dsigma, ddsigma);
        e = x_ref - x;
        eDot = dx_ref - dx;
        J_inv = computePinv(lag.J, eps, lambda);

        //state_ref.dq = J_inv * (dx_ref + 0.1*Kp.asDiagonal() * (x_ref - x));
        //state_ref.ddq = J_inv * (ddx_d - lag.JDot * state_ref.dq) + ((MatrixXd::Identity(state.q.size(), state.q.size()) - J_inv * lag.J)) * (-knd * state.dq);
        v = Kp.array() * e.array().abs().pow(alpha) * sat(e, 0).array();
        //state_ref.dq = J_inv * (dx_ref + 0.1*v);
        state_ref.dq = J_inv * (dx_ref + 0.05 * v + 0.05 * 1 * Kp.asDiagonal() * e);
        vDot = alpha * Kd.array() * e.array().abs().pow(alpha - 1) * eDot.array();
        state_ref.ddq = J_inv * (ddx_ref + Kp.asDiagonal() * e + 1 * Kd.asDiagonal() * eDot + vDot - lag.JDot * state_ref.dq);
        lag.update(state, state_ref); //update again for state_ref to get YY

        s = state.dq - state_ref.dq;     //sliding manifold
        s_d = s - delta * sat(s, delta); //manifold with boundary layer

        aDot = -1 * Ka.asDiagonal() * lag.Y.transpose() * s_d; //Adaptation law

        a = a + rate1 * dt * aDot;                             //integrate the estimated dynamic parameters parameters
        bDot = s_d.array().abs();
        b = b + rate2 * dt * Kb.asDiagonal() * bDot;
        //avoid_drifting(); // keep the dynamic parameters within range
        for(int i = 1; i < 11; i++){
            if (a(i) <= 0.0001){
                a(i) = 0.0001;
            }
        }
        for (int i = 0; i < 4; i++){
            if (b(i) <= 0.0001){
                b(i) = 0.0001;
            }
        }                      
        //cout << "\na \n " << a << "\n\n";i
        //cout << "\nb \n " << b << "\n\n";
        Ainv = computePinv(lag.A, 0.05, 0.05);                             // compute pesudoinverse of mapping matrix
        tau = Ainv * lag.Y * a -zz*(gamma * s - b.asDiagonal() * sat(s, delta)); // compute the desired toque in xy
        VectorXd pxy = stm->A_pseudo.inverse() * tau / 100;
        //pxy[1] += 300 * sin(sigma / 0.12 + 0);
        //pxy[3] += 300 * sin(sigma / 0.12 + 0);
        d_pxy = pxy - pprev;
        d_pxy = 15*sat(d_pxy,15);
        pxy = pprev + d_pxy;
        
        p = stm->pseudo2real(pxy);           // compute the desired pressure
        pprev = pxy;
        //auto end_tot = std::chrono::system_clock::now();
        //std::chrono::duration<double> elapsed_mod = end_mod - start_mod;
        //std::chrono::duration<double> elapsed_tot = end_tot - start_tot;
        //model_time = elapsed_mod.count();
        //tot_time = elapsed_tot.count();
        
        actuate(p);                                                          // control the valves
        if (fast_logging){
            int c_r = (int) (t/dt); //current row
            log_matrix(c_r,0) = t;
            for (int i = 0; i < 3; i++){
                log_matrix(c_r, 1+i) = x(i);
                log_matrix(c_r, 4+i) = x_ref(i);
            }
            for (int i = 0; i < 11; i++){
                log_matrix(c_r,7+i) = a(i);
            }
            for (int i = 0; i < st_params.q_size; i++){
                log_matrix(c_r,18+i) = b(i);
            }
            for (int i = 0; i < st_params.q_size; i++){
                log_matrix(c_r,22+i) = state.q(i);
            }
            for (int i = 0; i < 3*st_params.num_segments; i++){
                log_matrix(c_r,26+i)= p(i);
            }
            for (int i = 0; i < st_params.q_size; i++){
                log_matrix(c_r,32+i) = pxy(i);
            }
        }
        if (!pause){
        t+=dt;
        t_internal+=dt;
        }
        
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
    //double r;
    for (int idx = 0; idx < state.q.size()/2; idx++)
    {
        //r = std::fmod(std::abs(state.q[idx]), 6.2831853071795862); //remainder of division by 2*pi
        state.q[2*idx+1] = std::max(eps_custom,std::abs(state.q[2*idx+1]))*sign(state.q[2*idx+1]);;
    }
}

void Adaptive::avoid_drifting() //keeps the dynamic parameters within the range
{
    for (int i = 0; i < Ka.size(); i++)
    {
        if (a(i) < a_min(i) || a(i) > a_max(i))
        {
            this->Ka(i) = 0;
            a(i) = std::min(a_max(i), std::max(a(i), a_min(i)));
        }
        else
            this->Ka(i) = Ka_(i);
    }
    for (int i = 0; i < Kb.size(); i++)
    {
        if (b(i) < b_min(i) || b(i) > b_max(i))
        {
            this->Kb(i) = 0;
            b(i) = std::min(b_max(i), std::max(b(i), b_min(i)));
        }
        else
            this->Kb(i) = Kb_(i);
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

void Adaptive::increase_gamma()
{
    this->gamma = 1.1 * this->gamma;
    fmt::print("gamma = {}\n", gamma);
}

void Adaptive::decrease_gamma()
{
    this->gamma = 0.9 * this->gamma;
    fmt::print("gamma = {}\n", gamma);
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

void Adaptive::increase_rate1()
{
    this->rate1 = 1.1 * this->rate1;
    fmt::print("rate1 = {}\n", rate1);
}

void Adaptive::decrease_rate1()
{
    this->rate1 = 0.9 * this->rate1;
    fmt::print("rate1 = {}\n", rate1);
}

void Adaptive::increase_rate2()
{
    this->rate2 = 1.1 * this->rate2;
    fmt::print("rate2 = {}\n", rate2);
}

void Adaptive::decrease_rate2()
{
    this->rate2 = 0.9 * this->rate2;
    fmt::print("rate2 = {}\n", rate2);
}

void Adaptive::increase_eps()
{
    this->eps_custom = 1.1 * this->eps_custom;
    fmt::print("eps_custom = {}\n", eps_custom);
}

void Adaptive::decrease_eps()
{
    this->eps_custom = 0.9 * this->eps_custom;
    fmt::print("eps_custom = {}\n", eps_custom);
}

void Adaptive::increase_stiffness(int seg)
{
    //this->a[9 + seg] = this->a[9 + seg] * 1.1;
    //std::cout << this->rate1;
    //std::cout << this->Ka(9);
    //std::cout << this->Ka(10);
    //fmt::print("Ka = {}\n", Ka);
    //fmt::print("aDot = {}\n", aDot);
    fmt::print("a = {}\n", a);
    fmt::print("b = {}\n", b);
    fmt::print("T = {}\n", T);
    //fmt::print("pressure = {}\n", d_pxy);
    //fmt::print("stiffness{}: {}\n", seg, this->a[9 + seg]);
    //fmt::print("damping: {}\n", this->a[7]);
    //fmt::print("damping: {}\n", this->a[8]);
}

void Adaptive::decrease_stiffness(int seg)
{
    this->a[9 + seg] = this->a[9 + seg] * 0.9;
    fmt::print("stiffness{}: {}\n", seg, this->a[9 + seg]);
}

void Adaptive::increase_damping()
{
    this->a[7] = this->a[7] * 1.1;
    this->a[8] = this->a[8] * 1.1;
    fmt::print("damping: {}\n", this->a[8]);
}

void Adaptive::decrease_damping()
{
    this->a[7] = this->a[7] * 0.9;
    this->a[8] = this->a[8] * 0.9;
    fmt::print("damping: {}\n", this->a[8]);
}

void Adaptive::increase_alpha()
{
    this->alpha = this->alpha * 1.1;
    fmt::print("alpha: {}\n", this->alpha);
}

void Adaptive::decrease_alpha()
{
    this->alpha = this->alpha * 0.9;
    fmt::print("alpha: {}\n", this->alpha);
}
void Adaptive::change_ref1()
{
    x_ref << 0.12, 0.0, -0.22;
    this->set_ref(x_ref, dx_ref, ddx_ref);
    fmt::print("position_changed = {}\n", x_ref);
}
void Adaptive::change_ref2()
{
    x_ref << 0.0, 0.12, -0.22;
    this->set_ref(x_ref, dx_ref, ddx_ref);
    fmt::print("position_changed = {}\n", x_ref);
}
void Adaptive::change_ref3()
{
    x_ref << -0.12, 0.0, -0.22;
    this->set_ref(x_ref, dx_ref, ddx_ref);
    fmt::print("position_changed = {}\n", x_ref);
}
void Adaptive::change_ref4()
{
    x_ref << 0.0, -0.12, -0.22;
    this->set_ref(x_ref, dx_ref, ddx_ref);
    fmt::print("position_changed = {}\n", x_ref);
}

void Adaptive::show_x()
{
    fmt::print("desried_position = {}\n", x_ref);
    //fmt::print("qlysis_position = {}\n", x_qualisys);
    fmt::print("FK_position = {}\n", x);
}

void Adaptive::start_AD()
{
    fmt::print("Adaptive Controller is activated!\n");
    this->rate1 = 0.001;
    this->rate2 = 0.00001;
    this->zz = 1;
    this->pause = false;
}
void Adaptive::start_ID()
{
    fmt::print("Inverse Dynamics Controller is activated!\n");
    this->rate1 = 0;
    this->rate2 = 0;
    this->gamma = 0;
    this->zz = 0;
}

void Adaptive::toggle_fastlog(double time){
    if (!fast_logging){
        fast_logging = true;
        log_matrix = MatrixXd::Zero((int) ((time+1)/dt),36);
        t = 0;
    } else {
        fast_logging = false;
        filename = fmt::format("{}/adaptive_fastlog.csv", SOFTTRUNK_PROJECT_DIR);
        fmt::print("Dumping memory log to {}\n", filename);
        log_file.open(filename, std::fstream::out);
        log_file << "t,x,y,z,x_ref,y_ref,z_ref,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,b1,b2,b3,b4,q0,q1,q2,q3,p0,p1,p2,p3,p4,p5,tau1,tau2,tau3,tau4\n";
        for (int i = 0; i < log_matrix.rows(); i ++){
            for (int j = 0; j < 36; j++){
                log_file << log_matrix(i,j) << ",";
            }
            log_file << "\n";
        }
        log_file.close();
    }
}

// Timing Law
void Adaptive::s_trapezoidal_speed(double t, double *sigma, double *dsigma, double *ddsigma, double *T)
{

    double l, dsigma_max, ddsigma_max, Ts, Tf;

    // circle
    double n = 2; //rounds of circle
    double r = 0.12;    //radius of the circle
    l = 2 * PI * n * r; //l > v_max ^ 2 / a_max

    // linear
    /*
    p_i = target_points[0 + target_point]; //initial point
    p_f = target_points[1 + target_point];//final point
    Eigen::Vector3d d = p_f - p_i;
    l = d.norm();
    //fmt::print("p_i =  {}\n", p_i);
    //fmt::print("p_f =  {}\n", p_f);
    */
    dsigma_max = 0.05;  // maximum velocity
    ddsigma_max = 0.01; // maximum acc
    double l_min =  dsigma_max * dsigma_max / ddsigma_max;
    //fmt::print("L =  {}\n", l);
    //fmt::print("l_min =  {}\n", l_min);
    assert (l > l_min);

    Ts = dsigma_max / ddsigma_max;
    Tf = (l * ddsigma_max + (dsigma_max * dsigma_max)) / (ddsigma_max * dsigma_max); // the total time
    *T = Tf;

    if (t <= Ts)
    {

        *sigma = (ddsigma_max * t * t) / 2;
        *dsigma = ddsigma_max * t;
        *ddsigma = ddsigma_max;
    }

    else if (t > Ts && t <= Tf - Ts)
    {

        *sigma = (dsigma_max * t) - ((dsigma_max * dsigma_max) / (2 * ddsigma_max));
        *dsigma = dsigma_max;
        *ddsigma = 0;
    }

    else if (t > Tf - Ts && t <= Tf)
    {

        *sigma = (-0.5 * ddsigma_max * (t - Tf) * (t - Tf)) + (dsigma_max * Tf) - (dsigma_max * dsigma_max / ddsigma_max);
        *dsigma = ddsigma_max * (Tf - t);
        *ddsigma = -ddsigma_max;
    }

    else if (t > Tf)
    {
        t = Tf;
        *sigma = (-0.5 * ddsigma_max * (t - Tf) * (t - Tf)) + (dsigma_max * Tf) - (dsigma_max * dsigma_max / ddsigma_max);
        *dsigma = 0;  //;ddsigma_max*(Tf-t);
        *ddsigma = 0; //-ddsigma_max;
    }
}

void Adaptive::Task_Circle_r2r(double sigma, double dsigma, double ddsigma)
{
    // circle parameters
    double cx = 0;
    double cy = 0.0;
    double cz = -0.235;
    double r = 0.12;
    double h = -0.02;
    double phi = 0;
    double psi = 0;
    this->x_ref[0] = cx + r * cos(sigma / r + phi);
    this->x_ref[1] = cy + r * sin(sigma / r + phi);
    this->x_ref[2] = cz + h * cos(sigma / r + psi);

    this->dx_ref[0] = -sin(sigma / r + phi) * dsigma;
    this->dx_ref[1] = cos(sigma / r + phi) * dsigma;
    this->dx_ref[2] = -(h * sin(psi + sigma / r)) / r * dsigma;

    this->ddx_ref[0] = (-cos(phi + sigma / r) / r) * dsigma * dsigma + (-sin(sigma / r + phi)) * ddsigma;
    this->ddx_ref[1] = (-sin(phi + sigma / r) / r) * dsigma * dsigma + (cos(sigma / r + phi)) * ddsigma;
    this->ddx_ref[2] = (-(h * cos(psi + sigma / r)) / (r * r)) * dsigma * dsigma + (-(h * sin(psi + sigma / r)) / r) * ddsigma;
}

void Adaptive::Task_Linear_r2r(double sigma, double dsigma, double ddsigma)
{
    //assert(target_point + 2 <= target_points.size());
    //p_i = target_points[0 + target_point]; //initial point
    //p_f = target_points[1 + target_point];//final point

    Vector3d d = p_f - p_i;
    double L = d.norm(); //distance

    this->x_ref[0] = p_i(0) + sigma*(p_f(0)-p_i(0))/L;
    this->x_ref[1] = p_i(1) + sigma*(p_f(1)-p_i(1))/L;
    //this->x_ref[2] = p_i(2) + sigma*(p_f(2)-p_i(2))/L;

    this->dx_ref[0] = (p_f(0)-p_i(0))/L * dsigma;
    this->dx_ref[1] = (p_f(1)-p_i(1))/L * dsigma;
    //this->dx_ref[2] = (p_f(2)-p_i(2))/L * dsigma;

    this->ddx_ref[0] = (p_f(0)-p_i(0))/L * ddsigma;
    this->ddx_ref[1] = (p_f(1)-p_i(1))/L * ddsigma;
    //this->ddx_ref[2] = (p_f(2)-p_i(2))/L * ddsigma;

     double r = sqrt(pow(this->x_ref(0),2) + pow(this->x_ref(1),2));
    this->x_ref(2) = 5.89*pow(r,3) + 1.21*pow(r,2) - 0.00658*r -0.2737; //this polynomial obtained from task space fit, view Oliver Fischer -> SoPrA General -> Task Space Analyis
    this->dx_ref(2) = 0;
    this->ddx_ref(2) = 0;
   
    if (t_internal >= T){
        target_point += 1;
        t_internal = 0;
    }
}

void Adaptive::coordinates_S (double x_max, double y_max, double y_mid, double default_z) {
    /* Shape defined by 6 waypoints as follows:
    2(-x_max,  y_max) --- 1(x_max,  y_max) 
    | 
    |
    3(-x_max,  y_mid) --- 4(x_max,  y_mid) 
                                         |
                                         |
    6(-x_max, -y_max) --- 5(x_max, -y_max) 
    */

    target_points.resize(6);
    
    target_points[0] << x_max,  y_max, default_z;
    target_points[1] << -x_max, y_max, default_z;
    target_points[2] << -x_max, y_mid, default_z;
    target_points[3] << x_max,  y_mid, default_z;
    target_points[4] << x_max, -y_max+y_mid, default_z;
    target_points[5] << -x_max, -y_max+y_mid, default_z;
}



void Adaptive::coordinates_R(double x_max, double y_max, double y_mid, double default_z) {
    /* Shape defined by 6 waypoints as follows:
    2(-x_max,  y_max) --- 3(x_max,  y_max) 
    |                                    |
    |                                    |
    5(-x_max,  y_mid) --- 4(x_max,  y_mid) 
    |                 \                   
    |                   \          
    1(-x_max, -y_max) --- 6(x_max, -y_max) 
    */
    target_points.resize(6);
    
    
    target_points[0] << -x_max, -y_max, default_z;
    target_points[1] << -x_max,  y_max, default_z;
    target_points[2] << x_max,  y_max, default_z;
    target_points[3] << x_max,  y_mid, default_z;
    target_points[4] << -x_max, -y_mid, default_z;
    target_points[5] << x_max, -y_max, default_z;

}



void Adaptive::coordinates_L (double x_max, double y_max, double default_z) {
    /* Shape defined by 3 waypoints as follows:
    1(-x_max,  y_max) 
    | 
    |
    2(-x_max, -y_max) --- 3(x_max, -y_max) 
    */
    target_points.resize(3);
    
    target_points[0] << -x_max,  y_max, default_z;
    target_points[1] << -x_max, -y_max, default_z;
    target_points[2] << x_max, -y_max, default_z;

}

