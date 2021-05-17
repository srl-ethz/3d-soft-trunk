//
// Created by yasu on 25/12/2020.
//

#include "3d-soft-trunk/ControllerPCC.h"

/**
 * @brief run a PID controller. Set new reference pose every time you press ENTER,
 */
int main(){
    VectorXd p;
    srl::State state;

    Vector3d x_ref_center;
    Vector3d x_ref;
    x_ref_center << 0,0.1,-0.2;
    Vector3d dx_ref = Vector3d::Zero();
    
    double t = 0;
    double dt = 0.2;
    Vector3d circle;

    double amplitude = 0.05;
    double coef = 2 * 3.1415 / 4;




    ControllerPCC cpcc{CurvatureCalculator::SensorType::qualisys};
    getchar();
    cpcc.set_ref(x_ref_center, dx_ref);
    x_ref = x_ref_center;
    getchar();
    while (true) {
        /*fmt::print("~~~~press ENTER to set reference to current pose~~~~\n");
        getchar();
        cpcc.get_state(state);
        cpcc.set_ref(state);*/
        
        cpcc.get_pressure(p);
        cpcc.get_state(state);
        
        circle << cos(coef*t), 0, sin(coef*t);
        x_ref = x_ref_center + amplitude*circle;
        circle << -sin(coef*t), 0, cos(coef*t);
        dx_ref = amplitude * coef * circle;

        cpcc.set_ref(x_ref, dx_ref);
        
        fmt::print("--------\n");
        fmt::print("target pos:  {}\n", x_ref.transpose());
        fmt::print("actual pos: {}\n", (cpcc.stm->ara->get_H_base().rotation()*cpcc.get_H(st_params::num_segments-1).translation()).transpose());
        cpcc.print_ddx();
        t+=dt;
        srl::sleep(dt);
    }

}
