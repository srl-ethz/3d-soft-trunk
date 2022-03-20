#include "3d-soft-trunk/Controllers/Characterizer.h"

int main(){ 
    SoftTrunkParameters st_params{};
    st_params.load_yaml("testing.yaml"); //load the YAML "testing.yaml"
    st_params.finalize();
    Characterize ch{st_params}; //create a characterizer using the loaded YAML
    ch.valveMap(st_params.p_max); //do the valvemap
    ch.new_params.write_yaml("testing.yaml"); //overwrite the YAML with the new valvemapping
    return 0;
}