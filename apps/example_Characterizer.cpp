#include "3d-soft-trunk/Characterize.h"

int main(){
    SoftTrunkParameters st_params{};
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    Characterize ch{st_params, CurvatureCalculator::SensorType::qualisys};
    //ch.calcK(1,4,5);
    ch.logRadialPressureDist(0, "topRadialLog500");
    
}