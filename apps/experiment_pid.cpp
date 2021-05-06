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
    
    ControllerPCC cpcc{CurvatureCalculator::SensorType::qualisys};

    while (true) {
        fmt::print("~~~~press ENTER to set reference to current pose~~~~\n");
        getchar();
        cpcc.get_state(state);
        cpcc.set_ref(state);

        cpcc.get_pressure(p);

        fmt::print("--------\n");
        fmt::print("q:\t{}\n", state.q.transpose());
        fmt::print("pressure:\t{}\n", p.transpose());
    }

}
