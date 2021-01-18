//
// Created by yasu on 22/10/18.
//

#include "CurvatureCalculator.h"

/**
 * @file example_CurvatureCalculator.cpp
 * @brief An example demonstrating the use of the CurvatureCalculator.
 * @details This demo prints out the current q continuously.
 */

int main() {
    CurvatureCalculator cc{};
    VectorXd q, dq, ddq;
    Rate r{5};
    while (true) {
        cc.get_curvature(q, dq, ddq);
        fmt::print("==========\nq:\t{}\ndq:\t{}\nddq:\t{}\n", q.transpose(), dq.transpose(), ddq.transpose());
        r.sleep();
    }
    return 1;
}