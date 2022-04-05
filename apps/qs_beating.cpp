#include "3d-soft-trunk/Controllers/QuasiStatic.h"

Vector3d x_ref;
Vector3d x;
Vector3d dx_ref;
Vector3d ddx_ref;

void gain(QuasiStatic& QuasiStatic){ //change parameters online with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                QuasiStatic.kp_*=1.1;
                break;
            case 'a':
                QuasiStatic.kp_*=0.9;
                break;
            case 'e':
                QuasiStatic.kd_*=1.1;
                break;
            case 'd':
                QuasiStatic.kd_*=0.9;
                break;
            case 'm':
                QuasiStatic.toggle_log();
                break;
        }
        fmt::print("kp = {}, kd = {}\n", QuasiStatic.kp_, QuasiStatic.kd_);
    }
}

void printer(QuasiStatic& QuasiStatic){ //print some stuff every once in a while
    srl::Rate r{0.3};
    while(true){
        fmt::print("------------------------------------\n");
        fmt::print("q: {}\n",QuasiStatic.state_.q.transpose());
        fmt::print("x tip: {}\n", QuasiStatic.x_.transpose());
        fmt::print("x target: {}\n", QuasiStatic.x_ref_.transpose());
        fmt::print("x error normalized: {}\n", (x_ref-QuasiStatic.x_).norm());
        r.sleep();
    }
}

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("stateoftheart.yaml");
    st_params.finalize();
    QuasiStatic QuasiStatic(st_params);
    VectorXd p;
    srl::State state = st_params.getBlankState();
    
    double t = 0;
    double dt = 0.05;

    QuasiStatic.gripperAttached_ = true;
    QuasiStatic.loadAttached_ = 0;

    getchar();
    std::thread print_thread(printer, std::ref(QuasiStatic));
    std::thread gain_thread(gain, std::ref(QuasiStatic));

    srl::sleep(0.1);


    x_ref << 0.265, 0.0, 0.0;

    QuasiStatic.set_ref(x_ref, dx_ref, ddx_ref);

    getchar();
    //now hit it
    QuasiStatic.toggle_log();

    srl::sleep(10);

    QuasiStatic.toggle_log();
    return 1;
}
