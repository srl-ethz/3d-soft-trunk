#include "3d-soft-trunk/SoftTrunk_common.h"
#include "3d-soft-trunk/ControllerPCC.h"
#include "3d-soft-trunk/Lagrange.h"

#include "Eigen/Dense"


int main(){
    SoftTrunkParameters st_params_l{};
    st_params_l.load_yaml("softtrunkparams_Lagrange.yaml");
    st_params_l.finalize();  // run sanity check and finalize parameters

    SoftTrunkParameters st_params{};
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    Lagrange lag(st_params_l);
    ControllerPCC cpcc(st_params, CurvatureCalculator::SensorType::qualisys);
    srl::State state = st_params_l.getBlankState(); // get blank state with appropriate size
    srl::State state_r = st_params_l.getBlankState();
    VectorXd p = VectorXd::Zero(6);
    VectorXd t = VectorXd::Zero(4);
    double dt = 1./100;
    srl::Rate r{1./dt};
    VectorXd k_a;
    VectorXd k_p;
    k_a << 1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.;
    k_p << 1.,1.,1.,1.;
    // theta_1 = 0.3;
    VectorXd q_des = VectorXd::Random(4);
    VectorXd dq_des = VectorXd::Random(4);
    VectorXd ddq_des = VectorXd::Random(4);

    state_r.dq  = dq_des + k_p.asDiagonal()* (q_des - state.q);
    state_r.ddq = ddq_des + k_p.asDiagonal()* (dq_des - state.dq);
    MatrixXd Y = lag.Y;
    t = k_a.asDiagonal() * Y.transpose() * (state_r.dq - state.dq);

    while(true){
    r.sleep();    
    cpcc.cc->get_curvature(state);
    lag.update(state,state_r);

    //da = 

    //a += da*dt;

    p = cpcc.stm->pseudo2real(cpcc.stm->A_pseudo.inverse()*t);
    cpcc.actuate(p);

    }
    return 1;
}