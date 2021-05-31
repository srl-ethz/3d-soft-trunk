#include "3d-soft-trunk/Characterize.h"

int main(){
    Characterize ch{CurvatureCalculator::SensorType::qualisys};
    ch.logRadialPressureDist(0, "topRadialLog300");
    ch.calcK(1,8,5);
    fmt::print("\n\n You can close the laptop now\n");
}