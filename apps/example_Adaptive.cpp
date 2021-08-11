#include "3d-soft-trunk/Adaptive.h"

bool freedom = false;
Vector3d x_ref;
Vector3d x;
Vector3d dx_ref = Vector3d::Zero();
Vector3d ddx_ref = Vector3d::Zero();
Vector3d circle = Vector3d::Zero();
Vector3d d_circle = Vector3d::Zero();
Vector3d dd_circle = Vector3d::Zero();

void gain(Adaptive &ad){ //change gain with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                ad.increase_kd();
                break;
            case 'a':
                ad.decrease_kd();
                break;
            case 'e':
                ad.increase_kp();
                break;
            case 'd':
                ad.decrease_kp();
                break;
            case 't':
                ad.increase_stiffness(0);
                break;
            case 'g':
                ad.decrease_stiffness(0);
                break;
            case 'z':
                ad.increase_stiffness(1);
                break;
            case 'h':
                ad.decrease_stiffness(1);
                break;
        }
    }
}


int main(){
    

    SoftTrunkParameters st_params{};
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    Adaptive ad(st_params, CurvatureCalculator::SensorType::qualisys, 1);

    double t = 0;
    double dt = 0.1;
    x_ref << -0.13, 0,-0.21;
    std::thread gain_thread(gain, std::ref(ad));
    double amplitude = 0.2;
    double coef = 2 * 3.1415 / 4;
    fmt::print("first direction\n");
    ad.set_ref(x_ref,dx_ref,ddx_ref);
    //srl::sleep(6);
    fmt::print("secnd direction\n");
    /*x_ref << 0.13, 0,-0.21;
    ad.set_ref(x_ref);
*/
    ad.toggle_log();
    while (true){
        
        double r = 0.12;
        circle << r*cos(coef*t), r*sin(coef*t), -0.2;
        d_circle << -r*coef*sin(coef*t), r*coef*cos(coef*t),0;
        dd_circle << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t),0;
        x_ref = circle;
        dx_ref = d_circle;
        ddx_ref = dd_circle;
        //std::cout << "ref" << x_ref << "\n";
        //x_ref = ad.get_objects()[0];
        //ad.set_ref(x_ref,dx_ref,ddx_ref);
        
        t+=dt;
        srl::sleep(dt);
    }
    ad.toggle_log();

    srl::sleep(2);
}