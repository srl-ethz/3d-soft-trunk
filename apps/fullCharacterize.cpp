#include "3d-soft-trunk/Controllers/Characterizer.h"

int main(){

    //first, determine valve mapping
    if(true){ //this if statement allows us to destroy everything after doing characterization
    SoftTrunkParameters st_params{};
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    Characterize ch{st_params};
    ch.valveMap(st_params.p_max);
    ch.new_params.write_yaml("fullCharacterize.yaml");
    }

    //now, calculate stiffnesses
    if(true){ //this if statement allows us to destroy everything after doing characterization
    SoftTrunkParameters st_params{};
    st_params.load_yaml("fullCharacterize.yaml");
    st_params.finalize();
    Characterize ch{st_params};
    for (int i = 0; i < st_params.num_segments; i++){
        ch.stiffness(i,2,st_params.p_max);
    }
    ch.new_params.write_yaml("fullCharacterize.yaml");
    }

    //finally, using stiffness, determine "true" A matrix
    if(true){ //this if statement allows us to destroy everything after doing characterization
    SoftTrunkParameters st_params{};
    st_params.load_yaml("fullCharacterize.yaml");
    st_params.finalize();
    Characterize ch{st_params};
    for (int i = 0; i < st_params.num_segments; i++){
        ch.actuation(i,st_params.p_max);
    }
    ch.new_params.write_yaml("fullCharacterize.yaml");
    }

    return 0;
}