#include <3d-soft-trunk/Model.h>

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("oliver verification.yaml");
    st_params.finalize();
    srl::State state = st_params.getBlankState();
    DynamicParams dyn;
    StateEstimator ste{st_params};
    Model mdl{st_params};
    while (true){
        srl::sleep(1);
        ste.get_state(state);
        mdl.set_state(state);
        mdl.get_dynamic_params(dyn);
        fmt::print("B:\n{}\n",dyn.B);
    }
    return 0;
}