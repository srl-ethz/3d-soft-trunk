//
// Created by yasu and rkk on 26/10/18.
//

#include "3d-soft-trunk/ControllerPCC.h"

/**
 * @brief implements a PID controller whose parameters are defined using the Ziegler-Nichols method.
 * @param Ku ultimate gain
 * @param period oscillation period (in seconds)
 * @return MiniPID controller
 */
MiniPID ZieglerNichols(double Ku, double period, double control_period) {
    // https://en.wikipedia.org/wiki/Ziegler–Nichols_method
    // use P-only control for now
    double Kp = 0.2 * Ku;
    double Ki = 0;//.4 * Ku / period * control_period;
    double Kd = 0;//.066 * Ku * period / control_period;
    return MiniPID(Kp, Ki, Kd);
}

ControllerPCC::ControllerPCC(CurvatureCalculator::SensorType sensor_type) {
    
    // set up PID controllers
    if (st_params::controller == ControllerType::pid) {
        for (int j = 0; j < st_params::num_segments; ++j){
            miniPIDs.push_back(ZieglerNichols(Ku[j], Tu[j], dt)); // for X direction
            miniPIDs.push_back(ZieglerNichols(Ku[j], Tu[j], dt)); // for Y direction
        }
    }

    K_p = VectorXd::Ones(st_params::q_size);
    K_d = VectorXd::Ones(st_params::q_size);

    stm = std::make_unique<SoftTrunkModel>();
    // +X, +Y, -X, -Y
    std::vector<int> map = {1,2,5,3,6,4};
    vc = std::make_unique<ValveController>("192.168.0.100", map, p_max);
    if (sensor_type == CurvatureCalculator::SensorType::bend_labs)
        cc = std::make_unique<CurvatureCalculator>(sensor_type, bendlabs_portname);
    else if (sensor_type == CurvatureCalculator::SensorType::qualisys)
        cc = std::make_unique<CurvatureCalculator>(sensor_type);

    control_thread = std::thread(&ControllerPCC::control_loop, this);
}

void ControllerPCC::set_ref(const srl::State &state_ref) {
    std::lock_guard<std::mutex> lock(mtx);
    // assign to member variables
    this->state_ref = state_ref;
    if (st_params::controller == ControllerType::lqr){
        updateLQR(state_ref);
        u0 = gravity_compensate(state_ref);
    }
    if (!is_initial_ref_received)
        is_initial_ref_received = true;
}

void ControllerPCC::set_ref(const Vector3d &x_ref, const Vector3d &dx_ref){
    std::lock_guard<std::mutex> lock(mtx);
    this->x_ref = x_ref;
    this->dx_ref = dx_ref;
    if (!is_initial_ref_received)
        is_initial_ref_received = true;
}

void ControllerPCC::get_state(srl::State &state) {
    std::lock_guard<std::mutex> lock(mtx);
    state = this->state;
}
void ControllerPCC::get_pressure(VectorXd& p){
    std::lock_guard<std::mutex> lock(mtx);
    p = this->p;
}

Eigen::Transform<double, 3, Eigen::Affine> ControllerPCC::get_H(int segment_id){
    std::lock_guard<std::mutex> lock(mtx);
    return stm->get_H(segment_id);
};


VectorXd ControllerPCC::gravity_compensate(srl::State state){
    assert(st_params::sections_per_segment == 1);
    VectorXd gravcomp = stm->A_pseudo.inverse() * (stm->g + stm->K * state.q);
    return gravcomp/100; //to mbar
}

void ControllerPCC::actuate(VectorXd f) { //actuates valves according to mapping from header
    for (int i = 0; i < 3*st_params::num_segments; i++){
        vc->setSinglePressure(i, f(i));
    }
}

void ControllerPCC::toggle_log(){
    if(!logging) {
        logging = true;
        initial_timestamp = cc->get_timestamp();
        this->filename = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, filename);
        fmt::print("Starting log to {}\n", this->filename);
        log_file.open(this->filename, std::fstream::out);
        log_file << "timestamp";

        //write header
        log_file << fmt::format(", x, y, z");

        for (int i=0; i < st_params::q_size; i++)
            log_file << fmt::format(", q_{}", i);

        log_file << "\n";
    } else {
        logging = false;
        fmt::print("Ending log to {}\n", this->filename);
        log_file.close();
    }
}

void ControllerPCC::control_loop() {
    srl::Rate r{1./dt};
    VectorXd f = VectorXd::Zero(st_params::num_segments * 2); //2d pseudopressures
    while (true) {
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        // first get the current state from CurvatureCalculator.
        cc->get_curvature(state);
        stm->updateState(state);

        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;
        
        // calculate output
        switch (st_params::controller)
        {
        case ControllerType::dynamic:
            f = stm->A_pseudo.inverse() * (stm->g + stm->K*state_ref.q + stm->c + stm->D*state_ref.dq 
                    + K_p.asDiagonal()*(state_ref.q - state.q) + K_d.asDiagonal()*(state_ref.dq - state.dq)); 
            p = stm->pseudo2real(f/100);
            break;
        case ControllerType::pid:
            for (int i = 0; i < 2 * st_params::num_segments; ++i)
                f[i] = miniPIDs[i].getOutput(state.q[i], state_ref.q[i]);
            p = stm->pseudo2real(f + gravity_compensate(state));
            break;
        case ControllerType::gravcomp:
            p = stm->pseudo2real(gravity_compensate(state));
            break;
        case ControllerType::lqr:
            fullstate << state.q, state.dq;
            fullstate_ref << state_ref.q, state_ref.dq;
            f = K*(fullstate_ref - fullstate)/100;
            p = stm->pseudo2real(f + u0);
            break;
        case ControllerType::osc:
            x = stm->ara->get_H_base().rotation()*stm->ara->get_H_tip().translation();
            dx = stm->J*state.dq;
            ddx_ref = kp*(x_ref - x) + kd*(dx_ref - dx);            //values are critically damped approach
            ddx_ref*=1.3;
            B_op = (stm->J*stm->B.inverse()*stm->J.transpose()).inverse();
            g_op = B_op*stm->J*stm->B.inverse()*stm->g;
            J_inv = stm->B.inverse()*stm->J.transpose()*B_op;
            
            f = B_op*ddx_ref;// + g_op;
            tau_null = -0.1*state.q*0;
            tau_ref = stm->J.transpose()*f /*+ stm->K * state.q*/ + stm->D * state.dq + (MatrixXd::Identity(st_params::q_size, st_params::q_size) - stm->J.transpose()*J_inv.transpose())*tau_null;

            p = stm->pseudo2real(stm->A_pseudo.inverse()*tau_ref)/100 + stm->pseudo2real(gravity_compensate(state));
            break;
        }

        if (logging) {
            log_file << (cc->get_timestamp() - initial_timestamp)/ 1.0e6;
            
            log_file << fmt::format(", {}, {}, {}", x(0), x(1), x(2));
            
            for (int i=0; i < st_params::q_size; i++)               //log q
                log_file << fmt::format(", {}", state.q(i));
            log_file << "\n";
        }

        // actuate robot
        actuate(p);
    }
}

void ControllerPCC::updateLQR(srl::State state){ 
    assert(st_params::controller == ControllerType::lqr);
    stm->updateState(state);
    // make and update x' = Ax+Bu matrices
    // x = [q, dq]
    // formulate using pseudopressures
    MatrixXd lqrA = MatrixXd::Zero(2*st_params::q_size, 2*st_params::q_size);
    MatrixXd lqrB = MatrixXd::Zero(2*st_params::q_size, 2*st_params::num_segments);
    MatrixXd R = 0.0001*MatrixXd::Identity(2*st_params::num_segments, 2*st_params::num_segments);
    MatrixXd Q = 100000*MatrixXd::Identity(2*st_params::q_size, 2*st_params::q_size);
    Q.block(st_params::q_size, st_params::q_size, st_params::q_size, st_params::q_size) *= 0.001; // reduce cost for velocity
    MatrixXd Binv = stm->B.inverse();

    lqrA << MatrixXd::Zero(st_params::q_size,st_params::q_size), MatrixXd::Identity(st_params::q_size, st_params::q_size), - Binv * stm->K, -Binv * stm->D;
    lqrB << MatrixXd::Zero(st_params::q_size, 2*st_params::num_segments), Binv*stm->A_pseudo;
    // for debugging
    fmt::print("lqrA\n{}\nlqrB\n{}\n", lqrA, lqrB);
    fmt::print("Q\n{}\nR\n{}\n", Q, R);
    solveRiccatiArimotoPotter(lqrA, lqrB, Q, R, K);
    fmt::print("K\n{}\n", K);
}

void ControllerPCC::solveRiccatiArimotoPotter(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, //this function stolen from https://github.com/TakaHoribe/Riccati_Solver
                               const MatrixXd &R, Eigen::MatrixXd &K) {

  const uint dim_x = A.rows();
  const uint dim_u = B.cols();

  // set Hamilton matrix
  Eigen::MatrixXd Ham = Eigen::MatrixXd::Zero(2 * dim_x, 2 * dim_x);
  Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

  // calc eigenvalues and eigenvectors
  Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);

  // extract stable eigenvectors into 'eigvec'
  Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * dim_x, dim_x);
  int j = 0;
  for (int i = 0; i < 2 * dim_x; ++i) {
    if (Eigs.eigenvalues()[i].real() < 0.) {
      eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
      ++j;
    }
  }

  // calc P with stable eigen vector matrix
  Eigen::MatrixXcd Vs_1, Vs_2;
  Vs_1 = eigvec.block(0, 0, dim_x, dim_x);
  Vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
  MatrixXd P  = (Vs_2 * Vs_1.inverse()).real();
  K = R.inverse()*B.transpose()*P;
}
