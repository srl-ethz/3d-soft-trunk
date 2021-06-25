
#include "3d-soft-trunk/OSC.h"

bool freedom = false;
Vector3d x_ref;
Vector3d x;
Vector3d dx_ref = Vector3d::Zero();

void gain(OSC& osc){ //change gain with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                osc.set_kp(osc.get_kp()*1.1);
                break;
            case 'a':
                osc.set_kp(osc.get_kp()*0.9);
                break;
            case 'e':
                osc.set_kd(osc.get_kd()*1.1);
                break;
            case 'd':
                osc.set_kd(osc.get_kd()*0.9);
                break;
            case 'i':
                osc.potfields[0].set_cutoff(osc.potfields[0].get_cutoff()*1.1);
                break;
            case 'k':
                osc.potfields[0].set_cutoff(osc.potfields[0].get_cutoff()*0.9);
                break;
            case 'o':
                osc.potfields[0].set_strength(osc.potfields[0].get_strength()*1.1);
                break;
            case 'l':
                osc.potfields[0].set_strength(osc.potfields[0].get_strength()*0.9);
                break;
            case 'g':
                x_ref = osc.get_objects()[0];
                osc.set_ref(x_ref, dx_ref);
                break;
            case 't':
                osc.toggleGripper();
                freedom = true;
                break;
            case 'v':
                x_ref(1) *= -1;
                osc.set_ref(x_ref,dx_ref);
                break;
            case 'r':
                osc.toggle_log();
                srl::sleep(5);
                osc.toggle_log();
                break;
        }
        fmt::print("kp = {}, kd = {}\n", osc.get_kp(), osc.get_kd());
        fmt::print("cutoff = {}, strength = {}\n", osc.potfields[0].get_cutoff(), osc.potfields[0].get_strength());
    }
}

void printer(OSC& osc){
    srl::Rate r{1};
    while(true){
        Vector3d x;
        osc.get_x(x);
        fmt::print("------------------------------------\n");
        fmt::print("extra object: {}\n", osc.get_objects()[0].transpose());
        fmt::print("x error: {}\n", (x_ref-x).transpose());
        fmt::print("x error normalized: {}\n", (x_ref-x).norm());
        VectorXd p;
        osc.get_pressure(p);
        fmt::print("pressure: {}\n", p.transpose());
        r.sleep();
    }
}

int main(){
    SoftTrunkParameters st_params;
    st_params.finalize();
    OSC osc(st_params, CurvatureCalculator::SensorType::qualisys, false, 1);
    VectorXd p;
    srl::State state = st_params.getBlankState();

    Vector3d x_ref_center;
    
    x_ref_center << 0,0.15,-0.2;
    x_ref = x_ref_center;
    
    
    double t = 0;
    double dt = 0.1;
    Vector3d circle;

    double amplitude = 0.2;
    double coef = 2 * 3.1415 / 32;
    osc.gripperAttached = true;
    
    getchar();
    osc.set_ref(x_ref, dx_ref);
    // arguments to pass by reference must be explicitly designated as so
    // https://en.cppreference.com/w/cpp/thread/thread/thread
    // std::thread print_thread(printer, std::ref(osc));
    std::thread gain_thread(gain, std::ref(osc));
    
    osc.toggle_log();
    while (true){
        /*double r = amplitude*sin(3*coef*t);
        circle << r*cos(coef*t), r*sin(coef*t), -sqrt(pow(0.27,2) - pow(1.2*r,2));
        x_ref = circle;
        //dx_ref = amplitude * coef * circle;*/
        x_ref = osc.get_objects()[0];
        osc.set_ref(x_ref,dx_ref);
        /*osc.get_x(x);
        if ((x_ref - x).norm() < 0.07){
            freedom = true;
            osc.toggleGripper();
        }*/
        
        t+=dt;
        srl::sleep(dt);
    }
    osc.toggle_log();
    srl::sleep(2);
    
    x_ref << 0.15,0,-0.2;
    osc.set_ref(x_ref,dx_ref);
    srl::sleep(4);
    x_ref << -0.15,0,-0.2;
    dx_ref << -10, 0, 0;
    osc.set_ref(x_ref,dx_ref);
    srl::sleep(0.3);
    osc.toggleGripper();
    dx_ref << 0, 0, 0;
    osc.set_ref(x_ref,dx_ref);
    
    return 1;
}