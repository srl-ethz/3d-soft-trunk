
#include "3d-soft-trunk/Controllers/IDCon.h"

bool freedom = false;
Vector3d x_ref;
Vector3d x;
Vector3d dx_ref;
Vector3d ddx_ref;

void gain(IDCon& idc){ //change parameters online with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                idc.kp*=1.1;
                break;
            case 'a':
                idc.kp*=0.9;
                break;
            case 'e':
                idc.kd*=1.1;
                break;
            case 'd':
                idc.kd*=0.9;
                break;

            case 't':
                idc.toggleGripper();
                break;
            case 'm':
                idc.toggle_log();
                break;
        }
        fmt::print("kp = {}, kd = {}\n", idc.kp, idc.kd);
    }
}

void printer(IDCon& idc){ //print some stuff every once in a while
    srl::Rate r{0.3};
    while(true){
        fmt::print("------------------------------------\n");
        fmt::print("q: {}\n",idc.state_.q.transpose());
        fmt::print("x tip: {}\n", idc.x_.transpose());
        fmt::print("x error: {}\n", (x_ref-idc.x_).transpose());
        fmt::print("x error normalized: {}\n", (x_ref-idc.x_).norm());
        r.sleep();
    }
}

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("fullCharacterize.yaml");
    st_params.finalize();
    IDCon idc(st_params);
    VectorXd p;
    srl::State state = st_params.getBlankState();


    getchar();
    // pressure, period
    idc.circle(500, 8);

    return 1;
}
