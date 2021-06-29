#include "3d-soft-trunk/Characterize.h"

int main(){
    SoftTrunkParameters st_params{};
    st_params.finalize();
    Characterize ch{st_params, CurvatureCalculator::SensorType::qualisys};
    ch.logRadialPressureDist(0, "topRadialLog500");
    
}