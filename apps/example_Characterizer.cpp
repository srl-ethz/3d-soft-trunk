#include "3d-soft-trunk/Characterize.h"

int main(){
    Characterize ch{CurvatureCalculator::SensorType::qualisys};
    ch.logRadialPressureDist(0, "radialDistTopSegment");
    ch.logRadialPressureDist(1, "radialDistBottomSegment");
    fmt::print("\n\n You can close the laptop now\n");
}