#include "3d-soft-trunk/Controllers/Characterizer.h"

int main(){
    SoftTrunkParameters st_params{};
    st_params.load_yaml("testing.yaml");
    st_params.finalize();
    Characterize ch{st_params};
    ch.stiffness(0);
    ch.stiffness(1);
    ch.new_params.write_yaml("testing.yaml");
    //ch.stiffness(0);
    return 0;
}