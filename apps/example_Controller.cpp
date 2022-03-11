#include "3d-soft-trunk/ControllerPCC.h"

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    ControllerPCC cpcc{st_params};
    while(true){
        fmt::print("q: {}\n",cpcc.state_.q.transpose());
        srl::sleep(1);
    }

    return 1;
}