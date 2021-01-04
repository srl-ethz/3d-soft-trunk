//
// Created by yasu on 21/12/2020.
//

#include "CurvatureCalculator.h"
#include "AugmentedRigidArm.h"

int main() {
    // obtain data from motion tracking system and update the rigid model visualization in real time.
    // run drake-visualizer as well
    CurvatureCalculator cc{SensorType::bend_labs};
    AugmentedRigidArm ara{};
    VectorXd q, dq, ddq;
    Rate r{30};
    while (true) {
        cc.get_curvature(q, dq, ddq);
        ara.update(q, dq);
        fmt::print("----------\n{}\n", q.transpose());
        r.sleep();
    }
    return 1;
}