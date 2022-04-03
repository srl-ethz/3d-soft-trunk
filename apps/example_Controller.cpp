#include "3d-soft-trunk/ControllerPCC.h"

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("stiffness_vertical_test.yaml");
    st_params.finalize();
    st_params.write_yaml("test.yaml");
    
    ControllerPCC cpcc{st_params};
    srl::sleep(1);
    while(true){ //continuously print the measured state
        fmt::print("q: {}\n",cpcc.state_.q.transpose());
        fmt::print("x: {}\n",cpcc.state_.tip_transforms[st_params.num_segments+st_params.prismatic].translation().transpose());
        fmt::print("g: {}\n",(cpcc.dyn_.A_pseudo.inverse() * (cpcc.dyn_.g) / 100).transpose());
        fmt::print("K: {}\n",(cpcc.dyn_.A_pseudo.inverse() * (cpcc.dyn_.K*cpcc.state_.q) / 100).transpose());
        fmt::print("Base transform: \n{}\n",cpcc.state_.tip_transforms[0].matrix());
        fmt::print("Tip transform: \n{}\n",cpcc.state_.tip_transforms[2].matrix());
        srl::sleep(1); 
    }

    return 1;

}