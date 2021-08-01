#include "3d-soft-trunk/Characterize.h"

int main(){
    SoftTrunkParameters st_params{};
    st_params.finalize();
    Characterize ch{st_params, CurvatureCalculator::SensorType::qualisys};
    ch.calcK(1,8,5,"K_bottom");
    ch.calcK(0,8,5,"K_top");
    //ch.logRadialPressureDist(0, "topRadialLog500");
    
}