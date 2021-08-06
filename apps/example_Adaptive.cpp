#include "3d-soft-trunk/SoftTrunk_common.h"
#include "3d-soft-trunk/ControllerPCC.h"
#include "3d-soft-trunk/Lagrange.h"




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
    srl::State state_des = st_params_l.getBlankState();
    VectorXd p = VectorXd::Zero(6);
    VectorXd t = VectorXd::Zero(4);
    while(true){
    cpcc.cc->get_curvature(state);
    lag.update(state);


    p = cpcc.stm->pseudo2real(cpcc.stm->A_pseudo.inverse()*t);
    cpcc.actuate(p);

    }
    return 1;
}