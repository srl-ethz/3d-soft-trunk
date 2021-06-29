#include <3d-soft-trunk/SoftTrunkModel.h>

/**
 * demo to showcase how to use the SoftTrunkModel class, as well as create a custom arm configuration youself. 
 */
int main(){
    SoftTrunkParameters st_params{};

    // edit parameters, create ridiculous-looking custom model
    st_params.robot_name = "myModel";
    st_params.num_segments = 3;
    st_params.sections_per_segment = 1;
    st_params.masses = {0.1,0.1,0.1,0.1,0.1,0.1};
    st_params.lengths = {0.2,0.05,0.2,0.05,0.2,0.05};
    st_params.diameters = {0.01, 0.1, 0.02, 0.01};
    st_params.armAngle = 30;
    st_params.shear_modulus = {30000, 30000, 30000};
    st_params.drag_coef = {10000, 10000, 10000};
    st_params.finalize();  // run sanity check and finalize parameters

    SoftTrunkModel stm = SoftTrunkModel(st_params);
    srl::State state = st_params.getBlankState();  // get blank state with appropriate size
    state.q(0) = 0.2;
    stm.updateState(state);
    // print parameters of model
    fmt::print("B:{}\nc:{}\ng:{}\nK:{}\nD:{}\nA:{}\nJ:{}\n", stm.B, stm.c, stm.g, stm.K, stm.D, stm.A, stm.J[st_params.num_segments-1]);

    // example of converting between pseudopressure to real pressure
    VectorXd v = VectorXd::Zero(2*st_params.num_segments);
    v(0) = 200;
    fmt::print("{}\n\n\n", stm.pseudo2real(v).transpose());

    return 1;
}
