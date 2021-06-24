#include "3d-soft-trunk/IKCon.h"

IKCon ik(CurvatureCalculator::SensorType::qualisys, false, 1);
Vector3d x_ref;
Vector3d dx_ref = Vector3d::Zero();


int main(){
    while(true){
        getchar();
        x_ref = ik.get_objects()[0];
        ik.set_ref(x_ref,dx_ref);
    }
    
}