#include "3d-soft-trunk/StateEstimator.h"
#include "3d-soft-trunk/Model.h"

/*
sample code for the StateEstimator class
- read from the sensor type defined for the SoftTrunkParams
- update the model based on the sensor readings (enabling visualization, etc.)

*/
int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();

    StateEstimator ste{st_params};
    srl::State state = st_params.getBlankState();
    Model mdl{st_params};

    srl::Rate r{50};
    while(true){ //continuously print state
        ste.poll_sensors();
        state = ste.state_;
        mdl.update(state);
        fmt::print("q: {}\n", state.q.transpose());
        r.sleep();
    }

    return 0;
}