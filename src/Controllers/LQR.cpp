#include "3d-soft-trunk/Controllers/LQR.h"

LQR::LQR(const SoftTrunkParameters st_params) : ControllerPCC::ControllerPCC(st_params){
    filename_ = "LQR_log";

    A = MatrixXd::Zero(2*st_params.q_size, 2*st_params.q_size);
    B = MatrixXd::Zero(2*st_params.q_size, 2*st_params.num_segments);

    R = 0.0001*MatrixXd::Identity(2*st_params.num_segments, 2*st_params.num_segments);
    Q = 100000*MatrixXd::Identity(2*st_params.q_size, 2*st_params.q_size);
    Q.block(st_params.q_size, st_params.q_size, st_params.q_size, st_params.q_size) *= 0.001; // reduce cost for velocity

    relinearize(); //linearizes in non-actuated position on startup

    control_thread_ = std::thread(&LQR::control_loop, this);
}

void LQR::relinearize(){    
    //update A, B with new dynamics
    A << MatrixXd::Zero(st_params_.q_size,st_params_.q_size), MatrixXd::Identity(st_params_.q_size, st_params_.q_size), - dyn_.B.inverse() * dyn_.K, -dyn_.B.inverse() * dyn_.D;
    B << MatrixXd::Zero(st_params_.q_size, 2*st_params_.num_segments), dyn_.B.inverse()*dyn_.A_pseudo;

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
    srl::Rate r{1./dt_};
    while(run_){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;
        
        //do controls

        fullstate << state_.q, state_.dq;
        fullstate_ref << state_ref_.q, state_ref_.dq;
        f_ = K*(fullstate_ref - fullstate)/100;

        p_ = mdl_->pseudo2real(f_ + gravity_compensate(state_));

        actuate(p_);
    }
}
