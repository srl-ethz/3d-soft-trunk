#include "3d-soft-trunk/Controllers/Characterizer.h"

int main(){ 
    SoftTrunkParameters st_params{};
    st_params.load_yaml("stiffness_vertical_test.yaml"); //load the YAML "testing.yaml"
    st_params.finalize();
    Characterize ch{st_params}; //create a characterizer using the loaded YAML
    ch.stiffness(0);
    ch.new_params.write_yaml("stiffness_vertical_test2.yaml"); //overwrite the YAML with the new valvemapping
    return 0;
}