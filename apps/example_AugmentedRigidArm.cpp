// Created by yasu on 26/10/18.
//

#include "3d-soft-trunk/Models/AugmentedRigidArm.h"
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

void q_update(double seconds, srl::State& state) {
    // generate nice-looking poses
    int dof = state.q.size();
    for (int i = 0; i < dof/2 ; i++) {
        state.q(2 * i + 0) = 1.6 * sin(seconds * (double) i / dof);
        state.q(2 * i + 1) = 0.8 * cos(seconds * (double) i / dof);

    }
}

int main() {
    SoftTrunkParameters st_params{};
    st_params.finalize();
    AugmentedRigidArm ara{st_params};

    // calculate the state of arm at a particular value of q and print out the various parameters
    srl::State state = st_params.getBlankState();

    double delta_t = 0.03;
    srl::Rate r{1. / delta_t};
    for (double t = 0; t<10; t+=delta_t) {
        q_update(t, state);
        ara.update(state);
        fmt::print("------------\n");
        std::cout << "q: " << state.q.transpose() << std::endl;
        // the rigid model's parameters are a too big to easily comprehend so view them in PCC parameter space
        std::cout << "B: " << ara.B << std::endl;
        std::cout << "g: " << ara.g << std::endl;
        std::cout << "J: " << ara.J[st_params.num_segments-1] << std::endl;
        std::cout << "H_tip: " << ara.get_H_tip().matrix() << std::endl;
        r.sleep();
    }

    fmt::print("switching to simulation mode...\n");
    ara.simulate();
    return 1;
}