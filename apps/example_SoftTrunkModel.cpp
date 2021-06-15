#include <3d-soft-trunk/SoftTrunkModel.h>

int main(){
    SoftTrunkParameters st_params{};
    st_params.finalize();
    SoftTrunkModel stm = SoftTrunkModel(st_params);
    // currently there's some errors when soft trunk is completely straight, so create some offset
    srl::State state = st_params.empty_state();
    /*state.q.setOnes();
    state.q *= 0.01;
    state.dq.setOnes();
    state.dq *= 0.01;*/
    stm.updateState(state);
    VectorXd v = VectorXd::Zero(4);
    v << 200, 0, 0, 0;
    fmt::print("{}\n\n\n", stm.pseudo2real(v).transpose());

    fmt::print("B:{}\nc:{}\ng:{}\nK:{}\nD:{}\nA:{}\nJ:{}\n", stm.B, stm.c, stm.g, stm.K, stm.D, stm.A, stm.J[st_params.num_segments-1]);
    return 1;
}
