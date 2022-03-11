#include "3d-soft-trunk/ControllerPCC.h"

int main(){
    SoftTrunkParameters st_params;
    st_params.finalize();
    ControllerPCC cpcc{st_params};
    while(true){
        fmt::print("q: {}\n",cpcc.state_.q.transpose());
    }

    return 1;
}