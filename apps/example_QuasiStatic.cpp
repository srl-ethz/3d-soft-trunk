#include "3d-soft-trunk/QuasiStatic.h"



bool freedom = false;
Vector3d x_ref;
Vector3d x;
Vector3d dx_ref;
Vector3d ddx_ref;

//for crossing
double crosstime;
Vector3d newx;
Vector3d diff;

void gain(QuasiStatic& qs){ //change gain with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                qs.set_kp(qs.get_kp()*1.1);
                break;
            case 'a':
                qs.set_kp(qs.get_kp()*0.9);
                break;
            case 'e':
                qs.set_kd(qs.get_kd()*1.1);
                break;
            case 'd':
                qs.set_kd(qs.get_kd()*0.9);
                break;
            case 'g':
                x_ref = qs.get_objects()[0];
                qs.set_ref(x_ref, dx_ref, ddx_ref);
                break;
            case 't':
                qs.toggleGripper();
                freedom = true;
                break;
            case 'v':
                //mirror across 0,0 in time "crosstime"
                crosstime = 1.5;
                ddx_ref = Vector3d::Zero();
                newx = x_ref;
                newx(1) *= -1;
                //newx(0) *= -1;
                //qs.toggle_log();
                /*diff = newx - x_ref;
                
                for(double tl = 0; tl < crosstime; tl += 0.1){
                    qs.set_ref(x_ref + (tl/crosstime) * diff, dx_ref, ddx_ref);
                    srl::sleep(0.1);
                }*/
                x_ref(0) = 0.17;
                x_ref(1) = 0;
                x_ref(2) = -0.27;
                qs.set_ref(x_ref,dx_ref,ddx_ref);
                //srl::sleep(1);
                x_ref = newx;
                qs.set_ref(x_ref,dx_ref,ddx_ref);
                //srl::sleep(8);
                //qs.toggle_log();
                break;

        }
        fmt::print("kp = {}, kd = {}\n", qs.get_kp(), qs.get_kd());
    }
}

void printer(QuasiStatic& qs){
    srl::Rate r{0.3};
    while(true){
        Vector3d x;
        qs.get_x(x);
        fmt::print("------------------------------------\n");
        fmt::print("extra object1: {}\n", qs.get_objects()[0].transpose());
        fmt::print("extra object1: {}\n", qs.get_objects()[1].transpose());
        fmt::print("x tip: {}\n", x.transpose());
        fmt::print("x error: {}\n", (x_ref-x).transpose());
        fmt::print("x error normalized: {}\n", (x_ref-x).norm());
        r.sleep();
    }
}

int main(){
    SoftTrunkParameters st_params;
    st_params.finalize();
    QuasiStatic qs(st_params, CurvatureCalculator::SensorType::qualisys, 2);
    VectorXd p;
    srl::State state = st_params.getBlankState();

    Vector3d x_ref_center;
    
    x_ref_center << 0.15*cos(0*0.01745329),0.15*sin(0*0.01745329),-0.215;
    //x_ref = x_ref_center;
    x_ref << 0.15,0.00,-0.2;
    std::thread print_thread(printer, std::ref(qs));

    
    double t = 0;
    double dt = 0.1;
    Vector3d circle;
    Vector3d d_circle;
    Vector3d dd_circle;

    double coef = 2 * 3.1415 / 8;
    qs.gripperAttached = true;
    qs.loadAttached = 0;
    getchar();
    qs.toggleGripper();

    getchar();
    qs.set_ref(x_ref, dx_ref, ddx_ref);
    getchar();
    // arguments to pass by reference must be explicitly designated as so
    // https://en.cppreference.com/w/cpp/thread/thread/thread
    std::thread gain_thread(gain, std::ref(qs));
    
    qs.toggle_log();
    while (false){
        double r = 0.13;
        circle << r*cos(coef*t), r*sin(coef*t),-0.215;
        d_circle << -r*coef*sin(coef*t), r*coef*cos(coef*t),0;
        dd_circle << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t),0;
        /*x_ref = circle;
        dx_ref = d_circle;
        ddx_ref = dd_circle;
        //x_ref = qs.get_objects()[0];
        //qs.set_ref(x_ref,dx_ref, ddx_ref);
        /*qs.get_x(x);
        if ((x_ref - x).norm() < 0.07){
            freedom = true;
            qs.toggleGripper();
        }*/
        
        t+=dt;
        srl::sleep(dt);
    }
    x_ref << -0.15,0.00,-0.2;
    dx_ref(0) = -10;
    qs.set_ref(x_ref,dx_ref);
    srl::sleep(0.2);
    qs.toggleGripper();
    srl::sleep(0.1);
    dx_ref(0) = 0;
    qs.set_ref(x_ref,dx_ref);
    srl::sleep(3);
    qs.toggle_log();
    srl::sleep(2);
    return 1;
}
