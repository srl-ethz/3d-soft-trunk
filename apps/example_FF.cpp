#include "3d-soft-trunk/ControllerPCC.h"

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();

    ControllerPCC cpcc(st_params, CurvatureCalculator::SensorType::qualisys);
    cpcc.set_log_filename("feedforward_log");
    VectorXd p = VectorXd::Zero(6);
    double time = 0;
    double dt = 0.01;
    double T = 18;
    srl::Rate r{1. /dt};
    cpcc.toggle_log();
    while(time < 2*T){
        
        for (int i = 0; i < 3; i++){
            cpcc.p(3+i) = 300*pow(sin(time*2*PI/T + i*2*PI/3),2);
        };
        for (int i = 0; i < 3; i++){
            cpcc.p(i) = 300*pow(sin(time*2*PI/T + i*2*PI/3),2);
        };
        /*
        cpcc.p(0) = 0;
        cpcc.p(1) = 600;
        cpcc.p(2) = 0;
        cpcc.p(3) = 0;
        cpcc.p(4) = 600;
        cpcc.p(5) = 0;
        */ 
        cpcc.cc->get_curvature(cpcc.state);
        cpcc.actuate(cpcc.p);
        time += dt;
        std::cout << time << "\n";
        r.sleep();
    };
    cpcc.toggle_log();
}
    
