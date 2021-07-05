#include "3d-soft-trunk/Characterize.h"

int main(){
    SoftTrunkParameters st_params{};
    st_params.finalize();
    Characterize ch{st_params, CurvatureCalculator::SensorType::qualisys};
    //ch.calcK(1,8,5);
    ch.logRadialPressureDist(0, "topRadialLog500");
    
}