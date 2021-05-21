#include "3d-soft-trunk/LQR.h"

LQR::LQR(CurvatureCalculator::SensorType sensor_type) : ControllerPCC::ControllerPCC(sensor_type){
    filename = "LQR_log";

    A = MatrixXd::Zero(2*st_params::q_size, 2*st_params::q_size);
    B = MatrixXd::Zero(2*st_params::q_size, 2*st_params::num_segments);

    R = 0.0001*MatrixXd::Identity(2*st_params::num_segments, 2*st_params::num_segments);
    Q = 100000*MatrixXd::Identity(2*st_params::q_size, 2*st_params::q_size);
    Q.block(st_params::q_size, st_params::q_size, st_params::q_size, st_params::q_size) *= 0.001; // reduce cost for velocity

    relinearize(state); //linearizes in non-actuated position on startup

    control_thread = std::thread(&LQR::control_loop, this);
}

void LQR::relinearize(srl::State state){
    stm->updateState(state);
    
    //update A, B with new dynamics
    A << MatrixXd::Zero(st_params::q_size,st_params::q_size), MatrixXd::Identity(st_params::q_size, st_params::q_size), - stm->B.inverse() * stm->K, -stm->B.inverse() * stm->D;
    B << MatrixXd::Zero(st_params::q_size, 2*st_params::num_segments), stm->B.inverse()*stm->A_pseudo;

    solveRiccati(A, B, Q, R, K);
}

void LQR::solveRiccati(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R, MatrixXd &K) {
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

    // calc P with matrix of stable eigenvectors
    Eigen::MatrixXcd Vs_1, Vs_2;
    Vs_1 = eigvec.block(0, 0, dim_x, dim_x);
    Vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
    MatrixXd P  = (Vs_2 * Vs_1.inverse()).real();

    K = R.inverse()*B.transpose()*P;
}


void LQR::control_loop() {
    srl::Rate r{1./dt};
    while(true){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        //update the internal visualization
        cc->get_curvature(state);
        stm->updateState(state); /* this is optional for LQR but allows arm to be visualized in drake-visualizer */ 

        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;
        
        //do controls

        fullstate << state.q, state.dq;
        fullstate_ref << state_ref.q, state_ref.dq;
        f = K*(fullstate_ref - fullstate)/100;

        p = stm->pseudo2real(f + gravity_compensate(state));

        actuate(p);
    }
}
