#include <3d-soft-trunk/SoftTrunkModel.h>

int main(){
    SoftTrunkModel stm = SoftTrunkModel();
    // currently there's some errors when soft trunk is completely straight, so create some offset
    srl::State state;
    state.q.setOnes();
    state.q *= 0.01;
    state.dq.setOnes();
    state.dq *= 0.01;
    stm.updateState(state);
    fmt::print("B:{}\nc:{}\ng:{}\nK:{}\nD:{}\nA:{}\nJ:{}\n", stm.B, stm.c, stm.g, stm.K, stm.D, stm.A, stm.J);
    return 1;
}