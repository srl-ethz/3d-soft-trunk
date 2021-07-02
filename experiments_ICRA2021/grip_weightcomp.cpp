#include "3d-soft-trunk/OSC.h"


bool freedom = false;
Vector3d x_ref;
Vector3d x;
Vector3d dx_ref;
Vector3d ddx_ref;

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
                osc.set_ref(x_ref, dx_ref, ddx_ref);
                break;
            case 't':
                assert(osc.gripperAttached);
                osc.toggleGripper();
                freedom = true;
                break;
            case 'v':
                x_ref(1) *= -1;
                osc.set_ref(x_ref,dx_ref, ddx_ref);
                break;
            case 'r':
                osc.toggle_log();
                srl::sleep(5);
                osc.toggle_log();
                break;
            case 'f':
                osc.set_log_filename("grip_diagram");
                osc.loadAttached = 0;
                osc.toggle_log();
                srl::sleep(1);
                osc.toggleGripper();
                osc.loadAttached = 0; //100grams
                srl::sleep(4);
                osc.toggle_log();
                osc.toggleGripper();
                osc.loadAttached = 0;
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
    OSC osc(st_params, CurvatureCalculator::SensorType::qualisys, 1);
    VectorXd p;
    srl::State state = st_params.getBlankState();
    
    x_ref << 0.,0.15,-0.2;
    osc.gripperAttached = true;
    
    getchar();
    osc.set_ref(x_ref, dx_ref, ddx_ref);
    std::thread gain_thread(gain, std::ref(osc));
    
    while (true){
    
    }


    
    return 1;
}