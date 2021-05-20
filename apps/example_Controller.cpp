
#include "3d-soft-trunk/OSC.h"

OSC osc(CurvatureCalculator::SensorType::qualisys);

/*void gain(){ //change gain with keyboard to avoid recompiling, a/y change kp and o/l change kd
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'a':
                cpcc.kp*=1.1;
                break;
            case 'y':
                cpcc.kp*=0.9;
                break;
            case 'o':
                cpcc.kd*=1.1;
                break;
            case 'l':
                cpcc.kd*=0.9;
                break;
        }
        fmt::print("kp = {}, kd = {}\n", cpcc.kp, cpcc.kd);
    }
}
*/
void printer(){
    srl::Rate r{1};
    while(true){
        //fmt::print("x = {}\n", cpcc.x.transpose());
        fmt::print("extra object: {}\n", osc.get_objects()[0].transpose());
        r.sleep();
    }
}

int main(){
    VectorXd p;
    srl::State state;

    Vector3d x_ref_center;
    Vector3d x_ref;
    x_ref_center << 0,-0.15,-0.2;
    x_ref = x_ref_center;
    Vector3d dx_ref = Vector3d::Zero();
    
    double t = 0;
    double dt = 0.2;
    Vector3d circle;

    double amplitude = 0.15;
    double coef = 2 * 3.1415 / 16;
    
    getchar();
    osc.set_ref(x_ref, dx_ref);
    std::thread print_thread(printer);
    //std::thread gain_thread(gain);
    
    osc.toggle_log();
    while (true) {
        
        /*circle << cos(coef*t), sin(coef*t), 0;
        x_ref = x_ref_center + amplitude*circle;
        circle << -sin(coef*t), cos(coef*t), 0;
        dx_ref = amplitude * coef * circle;*/
        getchar();
        x_ref = osc.get_objects()[0];
        osc.set_ref(x_ref, dx_ref);
        
        t+=dt;
        srl::sleep(dt);
    }
    osc.toggle_log();
    return 1;
}
