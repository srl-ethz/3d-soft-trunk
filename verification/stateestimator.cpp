#include "3d-soft-trunk/StateEstimator.h"

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("oliver verification.yaml");
    st_params.finalize();
    StateEstimator ste{st_params};
    srl::State state;
    while(true){
        srl::sleep(1);
        ste.get_state(state);
        fmt::print("q: {}\n",state.q.transpose());
    }
    return 1;
}