#include "3d-soft-trunk/Controllers/Characterizer.h"

int main(){
    SoftTrunkParameters st_params{};
        st_params.load_yaml("testing.yaml");
        st_params.finalize();
        Characterize ch{st_params};
        ch.actuation(0, 600);
        ch.actuation(1, 600);
        ch.new_params.write_yaml("testing.yaml");
            /*
    if(true){ //this if statement allows us to destroy everything after doing characterization
        SoftTrunkParameters st_params{};
        st_params.load_yaml("testing.yaml");
        st_params.finalize();
        Characterize ch{st_params};
        ch.valveMap();
        ch.new_params.write_yaml("testing.yaml");
    }
    if (true){
        SoftTrunkParameters st_params{};
        st_params.load_yaml("testing.yaml");
        st_params.finalize();
        Characterize ch{st_params};
        ch.stiffness(0);
        ch.stiffness(1);
        ch.new_params.write_yaml("testing.yaml");
    }
    */
}