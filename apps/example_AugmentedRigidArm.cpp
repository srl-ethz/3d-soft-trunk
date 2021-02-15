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
 */

void q_update(double seconds, VectorXd& q) {
    for (int i = 0; i < st_params::num_segments; i++) {
        if (st_params::parametrization == ParametrizationType::phi_theta){
            q(2*i + 0) = seconds + 1.1 * i;
            q(2*i + 1) = 0.35 + 0.3 * sin(seconds);
        }
        else if (st_params::parametrization == ParametrizationType::longitudinal){
            q(2 * i + 0) = sin(seconds * (1.5 + (double) i));
            q(2 * i + 1) = cos(seconds * (2.0 + (double) i));
        }
    }
}

int main() {
    AugmentedRigidArm ara{};

    // calculate the state of arm at a particular value of q and print out the various parameters
    VectorXd q = VectorXd::Zero(2 * st_params::num_segments);
    VectorXd dq = VectorXd::Zero(2 * st_params::num_segments);

    double delta_t = 0.03;
    srl::Rate r{1. / delta_t};
    for (double t = 0; t<10; t+=delta_t) {
        q_update(t, q);
        ara.update(q, dq);
        fmt::print("------------\n");
        fmt::print("q:{}\nxi:{}\n", q.transpose(), ara.xi.transpose());
        // the rigid model's parameters are a too big to easily comprehend so view them in PCC parameter space
        fmt::print("B:{}\n", ara.Jm.transpose() * ara.B_xi * ara.Jm);
        fmt::print("G:{}\n", (ara.Jm.transpose() * ara.G_xi).transpose());
        fmt::print("J:{}\n", ara.Jxi * ara.Jm);
        fmt::print("H_tip:{}\n", ara.get_H_tip().matrix());
        r.sleep();
    }

    fmt::print("switching to simulation mode...\n");
    ara.simulate();
    return 1;
}
