#include "3d-soft-trunk/StateEstimator.h"

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();

    StateEstimator ste{st_params};
    srl::State state = st_params.getBlankState();

    while(true){ //continuously print state
        ste.poll_sensors();
        state = ste.state_;
        fmt::print("q: {}\n", state.q.transpose());
        srl::sleep(1);
    }

    return 0;
}