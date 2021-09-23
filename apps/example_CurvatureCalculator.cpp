//
// Created by yasu on 22/10/18.
//

#include "3d-soft-trunk/Sensors/CurvatureCalculator.h"

/**
 * @file example_CurvatureCalculator.cpp
 * @brief An example demonstrating the use of the CurvatureCalculator.
 * @details This demo prints out the current q continuously.
 */

int main() {
    SoftTrunkParameters st_params{};
    st_params.finalize();
    CurvatureCalculator cc{st_params, CurvatureCalculator::SensorType::qualisys};
    srl::State state;
    unsigned long long int timestamp;
    srl::Rate r{5};
    while (true) {
        cc.get_curvature(state);
        timestamp = cc.get_timestamp();
        fmt::print("==========\nt:{}\tq:\t{}\ndq:\t{}\nddq:\t{}\n", timestamp, state.q.transpose(), state.dq.transpose(), state.ddq.transpose());
        r.sleep();
    }
    return 1;
}
