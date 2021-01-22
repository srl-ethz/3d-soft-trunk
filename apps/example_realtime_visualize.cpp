//
// Created by yasu on 21/12/2020.
//

#include "3d-soft-trunk/CurvatureCalculator.h"
#include "3d-soft-trunk/AugmentedRigidArm.h"

int main() {
    // obtain data from motion tracking system and update the rigid model visualization in real time.
    // run drake-visualizer as well
    CurvatureCalculator cc{CurvatureCalculator::SensorType::bend_labs, "/dev/cu.usbmodem14201"};
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