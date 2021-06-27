#include "3d-soft-trunk/IKCon.h"

IKCon ik(CurvatureCalculator::SensorType::qualisys, false, 1);
bool freedom = false;
Vector3d x_ref;
Vector3d x;
Vector3d dx_ref = Vector3d::Zero();
Vector3d ddx_ref = Vector3d::Zero();
Vector3d circle = Vector3d::Zero();
Vector3d d_circle = Vector3d::Zero();
Vector3d dd_circle = Vector3d::Zero();

void gain(){ //change gain with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                ik.set_kp(ik.get_kp()*1.1);
                break;
            case 'a':
                ik.set_kp(ik.get_kp()*0.9);
                break;
            case 'e':
                ik.set_kd(ik.get_kd()*1.1);
                break;
            case 'd':
                ik.set_kd(ik.get_kd()*0.9);
                break;
            case 'g':
                x_ref = ik.get_objects()[0];
                ik.set_ref(x_ref, dx_ref, ddx_ref);
                break;
            case 't':
                ik.toggleGripper();
                freedom = true;
                break;
            case 'v':
                x_ref(1) *= -1;
                ik.set_ref(x_ref,dx_ref, ddx_ref);
                break;
            case 'r':
                ik.toggle_log();
                srl::sleep(5);
                ik.toggle_log();
                break;
        }
        fmt::print("kp = {}, kd = {}\n", ik.get_kp(), ik.get_kd());
    }
}

void printer(){
    srl::Rate r{1};
    while(true){
        Vector3d x;
        ik.get_x(x);
        fmt::print("------------------------------------\n");
        fmt::print("extra object: {}\n", ik.get_objects()[0].transpose());
        fmt::print("x error: {}\n", (x_ref-x).transpose());
        fmt::print("x error normalized: {}\n", (x_ref-x).norm());
        VectorXd p;
        ik.get_pressure(p);
        fmt::print("pressure: {}\n", p.transpose());
        r.sleep();
    }
}

int main(){
    double t = 0;
    double dt = 0.1;
    x_ref << 0,0.15,-0.2;
    double amplitude = 0.2;
    double coef = 2 * 3.1415 / 32;
    bool freedom = false;
    srl::sleep(4);
    getchar();

    //std::thread print_thread(printer);
    std::thread gain_thread(gain);

    ik.toggle_log();
    while (t<32){

        double r = 0.15;
        circle << r*cos(coef*t), r*sin(coef*t), 0.2;
        d_circle << -r*coef*sin(coef*t), r*coef*cos(coef*t),0;
        dd_circle << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t),0;
        x_ref = circle;
        dx_ref = d_circle;
        ddx_ref = dd_circle;
        ik.set_ref(x_ref,dx_ref,ddx_ref);

        
        t+=dt;
        srl::sleep(dt);
    }
    ik.toggle_log();

    srl::sleep(2);
    
    
}