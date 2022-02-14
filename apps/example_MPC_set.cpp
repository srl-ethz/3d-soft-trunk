#include <3d-soft-trunk/ControllerPCC.h>
#include "3d-soft-trunk/MPC_constraints_finder.h"

#include <chrono>

int main(){

    SoftTrunkParameters st_params;
    st_params.finalize();
    MPC_constraints_finder mpc_set(st_params, CurvatureCalculator::SensorType::simulator);

    return 0; 
}