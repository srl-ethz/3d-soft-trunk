#include "3d-soft-trunk/Model.h"

int main(){
    SoftTrunkParameters st_params;
    //st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    
    Model mdl{st_params};

    srl::State state = st_params.getBlankState();
    DynamicParams dyn;

    for (int i = 0; i < st_params.num_segments+st_params.prismatic; i++) {
        state.q(2*i) = 0.3;
        state.q(2*i+1) = 0;
    }
    srl::Rate r{100.};
    mdl.update(state);
    dyn = mdl.dyn_;

    getchar();
    return 0;
}