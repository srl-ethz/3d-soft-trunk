#include "3d-soft-trunk/Controllers/Characterizer.h"

int main(){
    SoftTrunkParameters st_params{};
    st_params.finalize();
    Characterize ch{st_params};
    //ch.calcK(0,8,5);
    ch.logRadialPressureDist(0, "topRadialLog500");
    
}