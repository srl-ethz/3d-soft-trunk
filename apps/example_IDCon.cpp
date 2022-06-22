
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

    Vector3d x_ref_center;
    
    x_ref << 0.1,0.00,-0.22;

    
    double t = 0;
    double dt = 0.1;
    Vector3d circle;
    Vector3d d_circle;
    Vector3d dd_circle;

    double coef = 2 * 3.1415 / 8;

    getchar();
    idc.set_ref(x_ref, dx_ref, ddx_ref);
    srl::sleep(0.1);
    getchar();
        std::thread print_thread(printer, std::ref(idc));

    // arguments to pass by reference must be explicitly designated as so
    // https://en.cppreference.com/w/cpp/thread/thread/thread
    std::thread gain_thread(gain, std::ref(idc));
    
    while (true){ //circle
        double r = 0.1;
        circle << r*cos(coef*t), r*sin(coef*t),-0.22;
        d_circle << -r*coef*sin(coef*t), r*coef*cos(coef*t),0;
        dd_circle << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t),0;

        x_ref = circle;
        dx_ref = d_circle;
        ddx_ref = dd_circle;

        idc.set_ref(x_ref,dx_ref, ddx_ref);
        
        t+=dt;
        srl::sleep(dt);
    }

    return 1;
}
