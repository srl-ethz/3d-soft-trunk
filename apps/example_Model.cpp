#include "3d-soft-trunk/Model.h"

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    
    Model mdl{st_params};

    srl::State state = st_params.getBlankState();
    DynamicParams dyn;

    for (int i = 0; i < st_params.num_segments+st_params.prismatic; i++) { //random configuration
        state.q(2*i+st_params.prismatic) = 0.3;
        state.q(2*i+1+st_params.prismatic) = 0.1;
    }

    mdl.update(state);
    dyn = mdl.dyn_;

    fmt::print("Actuation: {}\n Inertia: {}\n Stiffness: {}\n Coriolis: {}\n Damping: {}\n Gravity: {}",dyn.A, dyn.B, dyn.K, dyn.c, dyn.D, dyn.g);

    getchar();
    return 0;
}