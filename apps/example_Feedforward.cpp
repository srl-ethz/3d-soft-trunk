#include "3d-soft-trunk/ControllerPCC.h"

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();

    ControllerPCC cpcc(st_params, CurvatureCalculator::SensorType::qualisys);
    cpcc.set_log_filename("feedforward_log");
    VectorXd p = VectorXd::Zero(6);
    double time = 0;
    srl::Rate r{1./0.1};
    cpcc.toggle_log();
    while(time < 16){

        for (int i = 0; i < 3; i++){
            cpcc.p(3+i) = 600*sin(time*2*PI/16 + i*2*PI/3);
        }
        cpcc.cc->get_curvature(cpcc.state);

        cpcc.actuate(cpcc.p);
        time += 0.1;
        r.sleep();
    }
    cpcc.toggle_log();
}