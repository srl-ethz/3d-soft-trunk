#include "3d-soft-trunk/Controllers/IDCon.h"

Vector3d x_ref;
Vector3d x;
Vector3d dx_ref;
Vector3d ddx_ref;

void gain(IDCon& IDCon){ //change parameters online with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                IDCon.kp_*=1.1;
                break;
            case 'a':
                IDCon.kp_*=0.9;
                break;
            case 'e':
                IDCon.kd_*=1.1;
                break;
            case 'd':
                IDCon.kd_*=0.9;
                break;
            case 'm':
                IDCon.toggle_log();
                break;
        }
        fmt::print("kp = {}, kd = {}\n", IDCon.kp_, IDCon.kd_);
    }
}

void printer(IDCon& IDCon){ //print some stuff every once in a while
    srl::Rate r{0.3};
    while(true){
        fmt::print("------------------------------------\n");
        fmt::print("q: {}\n",IDCon.state_.q.transpose());
        fmt::print("x tip: {}\n", IDCon.x_.transpose());
        fmt::print("x target: {}\n", IDCon.x_ref_.transpose());
        fmt::print("x error normalized: {}\n", (x_ref-IDCon.x_).norm());
        fmt::print("gravity: {}\n",IDCon.gravity_compensate(IDCon.state_).transpose());
        r.sleep();
    }
}

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("stiffness_vertical_test2.yaml");
    st_params.finalize();
    IDCon IDCon(st_params);
    VectorXd p;
    srl::State state = st_params.getBlankState();


    IDCon.gripperAttached_ = true;
    IDCon.loadAttached_ = 0;

    x_ref << 0.265, 0.0, 0.0;

    std::thread print_thread(printer, std::ref(IDCon));
    std::thread gain_thread(gain, std::ref(IDCon));
    IDCon.set_ref(x_ref, dx_ref, ddx_ref);

    srl::sleep(0.1);

    getchar();
    //now hit it
    IDCon.toggle_log();

    srl::sleep(10);

    IDCon.toggle_log();
    return 1;
}
