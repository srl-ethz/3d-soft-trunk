#include <3d-soft-trunk/SoftTrunkModel.h>

int main(){
    SoftTrunkModel stm = SoftTrunkModel();
    // currently there's some errors when soft trunk is completely straight, so create some offset
    Pose pose;
    pose.q = 0.01* VectorXd::Ones(2*st_params::num_segments*st_params::sections_per_segment);
    pose.dq = 0.01 * VectorXd::Ones(2*st_params::num_segments*st_params::sections_per_segment);
    stm.updateState(pose);
    fmt::print("B:{}\nc:{}\ng:{}\nK:{}\nD:{}\nA:{}\nJ:{}\n", stm.B, stm.c, stm.g, stm.K, stm.D, stm.A, stm.J);
    return 1;
}
