#include "3d-soft-trunk/Dyn.h"

int main(){
    srl::State state_ref;
    Dyn d(CurvatureCalculator::SensorType::qualisys, false); //warning: Dynamic is also an eigen function so make sure you use the namespace
    for (int i = 0; i < st_params::num_segments; i++){
        state_ref.q(2*i) = 0.3;
    }
    d.set_ref(state_ref);
    while(true);
}