#include "3d-soft-trunk/Adaptive.h"

bool freedom = false;
Vector3d x_ref;
Vector3d x;
Vector3d dx_ref = Vector3d::Zero();
Vector3d ddx_ref = Vector3d::Zero();
Vector3d circle = Vector3d::Zero();
Vector3d d_circle = Vector3d::Zero();
Vector3d dd_circle = Vector3d::Zero();


int main(){
    

    SoftTrunkParameters st_params{};
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    Adaptive ad(st_params, CurvatureCalculator::SensorType::qualisys, 1);

    double t = 0;
    double dt = 0.1;
    x_ref << 0.15,0,-0.2;
    double amplitude = 0.2;
    double coef = 2 * 3.1415 / 128;
    bool freedom = false;
    ad.set_ref(x_ref,dx_ref,ddx_ref);
    srl::sleep(3);

    ad.toggle_log();
    while (t<10){
        
        double r = 0.12;
        circle << r*cos(coef*t), r*sin(coef*t), -0.2;
        d_circle << -r*coef*sin(coef*t), r*coef*cos(coef*t),0;
        dd_circle << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t),0;
        x_ref = circle;
        dx_ref = d_circle;
        ddx_ref = dd_circle;
        //x_ref = ad.get_objects()[0];
        //ad.set_ref(x_ref,dx_ref,ddx_ref);
        
        
        t+=dt;
        srl::sleep(dt);
    }
    ad.toggle_log();

    srl::sleep(2);
}