#include "3d-soft-trunk/Controllers/Characterizer.h"

int main(){
    SoftTrunkParameters st_params{};
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    Characterize ch{st_params};
    //ch.valveMap();
    ch.stiffness(0);
    return 1;
}