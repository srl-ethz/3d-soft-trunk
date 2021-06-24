#include "3d-soft-trunk/IKCon.h"

IKCon ik(CurvatureCalculator::SensorType::qualisys, false, 1);
Vector3d x_ref;

Vector3d dx_ref = Vector3d::Zero();
Vector3d circle = Vector3d::Zero();

int main(){
    double t = 0;
    double dt = 0.1;
    x_ref << 0,0.15,-0.2;
    double amplitude = 0.2;
    double coef = 2 * 3.1415 / 32;
    while(true){
        double r = 0.1;
        circle << r*cos(coef*t), r*sin(coef*t), -sqrt(pow(0.27,2) - pow(1.2*r,2));
        x_ref = circle;
        ik.set_ref(x_ref,dx_ref);
        t+=dt;
        srl::sleep(dt);
    }
    
}