#include <3d-soft-trunk/SoftTrunkModel.h>

int main(){
    SoftTrunkModel stm = SoftTrunkModel();
    VectorXd q = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd dq = 0.01 * VectorXd::Ones(2*st_params::num_segments*st_params::sections_per_segment);
    stm.updateState(q, dq);
    fmt::print("B:{}\nC:{}\ng:{}\nK:{}\nD:{}\nA:{}\n", stm.B, stm.C, stm.g, stm.K, stm.D, stm.A);
    return 1;
}