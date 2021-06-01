#include "3d-soft-trunk/Characterize.h"

int main(){
    Characterize ch{CurvatureCalculator::SensorType::qualisys};
    ch.logRadialPressureDist(0, "topRadialLog500");
    
}