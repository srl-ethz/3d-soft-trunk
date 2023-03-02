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

    std::cout << "Actuation: " << dyn.A << std::endl;
    std::cout << "Inertia: " << dyn.B << std::endl;
    std::cout << "Stiffness: " << dyn.K << std::endl;
    std::cout << "Coriolis: " << dyn.c << std::endl;
    std::cout << "Damping: " << dyn.D << std::endl;
    std::cout << "Gravity: " << dyn.g << std::endl;

    getchar();
    return 0;
}