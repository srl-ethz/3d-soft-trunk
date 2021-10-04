#include "3d-soft-trunk/Model.h"

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    Model mdl{st_params, 100};
    srl::State state{CoordType::thetax, st_params.q_size};
    DynamicParams dyn;

    for (int i = 0; i < st_params.num_segments; i++) {
        state.q(2*i) = 0.3;
        state.q(2*i+1) = 0;
    }
    mdl.set_state(state);
    mdl.get_dynamic_params(dyn);
    srl::Rate r{100.};
    while(true) {
        r.sleep();
        fmt::print("state: {}\n",state.q);
        mdl.simulate(state, VectorXd::Zero(6), 0.001);

    }
    return 0;
}