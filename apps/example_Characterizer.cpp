#include "3d-soft-trunk/Characterize.h"

int main(){
    Characterize ch{CurvatureCalculator::SensorType::qualisys};
    ch.calcGravK(0,8);
}