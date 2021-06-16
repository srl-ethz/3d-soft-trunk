#include "3d-soft-trunk/Dyn.h"

int main(){
    SoftTrunkParameters st_params;
    st_params.finalize();
    srl::State state_ref = st_params.getBlankState();
    Dyn d(st_params, CurvatureCalculator::SensorType::qualisys, false); //warning: Dynamic is also an eigen function so make sure you use the namespace
    for (int i = 0; i < st_params.num_segments; i++){
        state_ref.q(2*i) = 0.3;
    }
    d.set_ref(state_ref);
    while(true);
}