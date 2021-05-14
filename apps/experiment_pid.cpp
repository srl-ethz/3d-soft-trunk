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
    Vector3d x_ref;
    x_ref << 0,0.1,-0.2;
    Vector3d dx_ref = Vector3d::Zero();
    ControllerPCC cpcc{CurvatureCalculator::SensorType::qualisys};

    while (true) {
        fmt::print("~~~~press ENTER to set reference to current pose~~~~\n");
        getchar();
        cpcc.get_state(state);
        cpcc.set_ref(x_ref, dx_ref);

        cpcc.get_pressure(p);

        fmt::print("--------\n");
        fmt::print("q:\t{}\n", state.q.transpose());
        fmt::print("pressure:\t{}\n", p.transpose());
    }

}
