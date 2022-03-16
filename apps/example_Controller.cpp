#include "3d-soft-trunk/ControllerPCC.h"

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    st_params.write_yaml("test.yaml");
    /*
    ControllerPCC cpcc{st_params};
    while(true){
        fmt::print("q: {}\n",cpcc.state_.q.transpose());
        fmt::print("x: {}\n",cpcc.state_.tip_transforms[2].translation().transpose());
        srl::sleep(1);
    }

    return 1;
    */
}