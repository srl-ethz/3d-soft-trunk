
#include "3d-soft-trunk/Controllers/OSC.h"

bool freedom = false;
Vector3d x_ref;
Vector3d x;
Vector3d dx_ref;
Vector3d ddx_ref;

void gain(OSC& osc){ //change parameters online with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                osc.kp_*=1.1;
                break;
            case 'a':
                osc.kp_*=0.9;
                break;
            case 'e':
                osc.kd_*=1.1;
                break;
            case 'd':
                osc.kd_*=0.9;
                break;
            case 'i':
                osc.potfields_[0].cutoff_distance_*=1.1;
                break;
            case 'k':
                osc.potfields_[0].cutoff_distance_*=0.9;
                break;
            case 'o':
                osc.potfields_[0].strength_*=1.1;
                break;
            case 'l':
                osc.potfields_[0].strength_*=0.9;
                break;
            case 'g':
                x_ref = osc.state_.objects[0].translation();
                osc.set_ref(x_ref);
                break;
            case 't':
                osc.toggleGripper();
                break;

            case 'b':
                osc.loadAttached_ = 0.2;
                break;
            case 'f': 
                osc.freeze = !osc.freeze;
                fmt::print("Freeze status: {}\n", osc.freeze);
                break;
            case 'm':
                osc.toggle_log();
                break;
        }
        fmt::print("kp = {}, kd = {}\n", osc.kp_, osc.kd_);
    }
}

void printer(OSC& osc){ //print some stuff every once in a while
    srl::Rate r{0.3};
    while(true){
        fmt::print("------------------------------------\n");
        fmt::print("q: {}\n",osc.state_.q.transpose());
        fmt::print("x tip: {}\n", osc.x_.transpose());
        fmt::print("x error: {}\n", (x_ref-osc.x_).transpose());
        fmt::print("x error normalized: {}\n", (x_ref-osc.x_).norm());
        fmt::print("gravity: {}\n",osc.gravity_compensate(osc.state_).transpose());
        r.sleep();
    }
}

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("stiffness_vertical_test.yaml");
    st_params.finalize();
    OSC osc(st_params);
    VectorXd p;
    srl::State state = st_params.getBlankState();

    x_ref << 0.255775, 0.0, -0.030;
    
    
    double t = 0;
    double dt = 0.1;
    Vector3d circle;
    Vector3d d_circle;
    Vector3d dd_circle;

    double coef = 2 * 3.1415 / 8;
    osc.gripperAttached_ = true;
    osc.loadAttached_ = 0;


    getchar();
        std::thread print_thread(printer, std::ref(osc));

    // arguments to pass by reference must be explicitly designated as so
    // https://en.cppreference.com/w/cpp/thread/thread/thread
    std::thread gain_thread(gain, std::ref(osc));

    while (true){ //circle
        // double r = 0.1;
        // circle << 0.2313413, r*cos(coef*t), r*sin(coef*t);
        // d_circle << -r*coef*sin(coef*t), r*coef*cos(coef*t),0;
        // dd_circle << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t),0;

        // x_ref = circle;
        // dx_ref = d_circle;
        // ddx_ref = dd_circle;

        // osc.set_ref(x_ref, dx_ref, ddx_ref);


        osc.set_ref(x_ref);
        
        t+=dt;
        srl::sleep(dt);
    }

    return 1;
}
