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
    st_params.load_yaml("stiffness_vertical_test.yaml");
    st_params.finalize();
    IDCon IDCon(st_params);
    VectorXd p;
    srl::State state = st_params.getBlankState();

    Vector3d x_ref_center;
    
    //x_ref << 0.1,0.00,-0.235;
    x_ref << 0.255, 0.0, 0;


    
    double t = 0;
    double dt = 0.1;
    Vector3d circle;
    Vector3d d_circle;
    Vector3d dd_circle;

    double coef = 2 * 3.1415 / 8;
    IDCon.gripperAttached_ = true;
    IDCon.loadAttached_ = 0;

    getchar();
    std::thread print_thread(printer, std::ref(IDCon));
    std::thread gain_thread(gain, std::ref(IDCon));
    IDCon.set_ref(x_ref, dx_ref, ddx_ref);
    srl::sleep(0.1);


    
    getchar();
    IDCon.toggle_log();
    t=0;
    while (t<16){ //circle
        double r = 0.05;
        //vertical configuration circle
        /* 
        circle << r*cos(coef*t), r*sin(coef*t),-0.235;
        d_circle << -r*coef*sin(coef*t), r*coef*cos(coef*t),0;
        dd_circle << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t),0;
        */


       // horizontal configuration circle
        circle << 0.255,-r*sin(coef*t), -0.05+r*cos(coef*t);
        d_circle << 0, r*coef*cos(coef*t), -r*coef*sin(coef*t);
        dd_circle << 0,  -r*coef*coef*sin(coef*t), -r*coef*coef*cos(coef*t);

        x_ref = circle;
        dx_ref = d_circle;
        ddx_ref = dd_circle;

        IDCon.set_ref(x_ref,dx_ref, ddx_ref);
        
        t+=dt;
        srl::sleep(dt);
    }
    IDCon.toggle_log();
    return 1;
}
