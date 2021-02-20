//
// Created by yasu on 26/10/18.
//

#include "3d-soft-trunk/AugmentedRigidArm.h"
#include <stdio.h>
#include <chrono>
#include <thread>

/**
 * @file example_AugmentedRigidArm.cpp
 * @brief demo of AugmentedRigidArm class.
 *
 * creates an augmented rigid arm model, then gives it some values (q and dq, the soft robot's configurations) so it can update its internal variables, then prints them out.
 * also updates the drake visualization, so run drake-visualization to see the rigid body model update itself in real time
 * 
 * The URDF model of the robot must be created first, which is currently done by SoftTrunkModel, so running this first will result in an error.
 */

void q_update(double seconds, VectorXd& q) {
    // generate nice-looking poses
    for (int i = 0; i < st_params::num_segments * st_params::sections_per_segment ; i++) {
        q(2 * i + 0) = 0.8 * sin(seconds * (double) i / st_params::sections_per_segment) / st_params::sections_per_segment;
        q(2 * i + 1) = 0.4 * cos(seconds * (double) i / st_params::sections_per_segment) / st_params::sections_per_segment;

    }
}

int main() {
    AugmentedRigidArm ara{};

    // calculate the state of arm at a particular value of q and print out the various parameters
    VectorXd q = VectorXd::Zero(2 * st_params::num_segments * st_params::sections_per_segment);
    VectorXd dq = VectorXd::Zero(2 * st_params::num_segments * st_params::sections_per_segment);

    double delta_t = 0.03;
    srl::Rate r{1. / delta_t};
    for (double t = 0; t<10; t+=delta_t) {
        q_update(t, q);
        ara.update(q, dq);
        fmt::print("------------\n");
        fmt::print("q:{}\n", q.transpose());
        // the rigid model's parameters are a too big to easily comprehend so view them in PCC parameter space
        fmt::print("B:{}\n", ara.B);
        fmt::print("g:{}\n", ara.g);
        fmt::print("J:{}\n", ara.J);
        fmt::print("H_tip:{}\n", ara.H_tip.matrix());
        r.sleep();
    }

    fmt::print("switching to simulation mode...\n");
    ara.simulate();
    return 1;
}
