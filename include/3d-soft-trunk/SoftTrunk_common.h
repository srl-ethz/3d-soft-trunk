/**
 * @file SoftTrunk_common.h
 * @brief defines various constants and convenience functions for the Soft Trunk that are used across different files.
 */

#pragma once

#include <mobilerack-interface/common.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <thread>
#include <assert.h>
#include <array>

using namespace Eigen;

enum class ControllerType {
    dynamic,
    pid,        // for PID control, error is directly converted to pressure (i.e. alpha not used)
    gravcomp,   //attempts to hold current position, make arm compliant
    lqr,        //LQR controller, directly to pressure
};



/** @brief parameters that define the configuration of the Soft Trunk */
namespace st_params {
    /** @brief name of robot (and of urdf / xacro file) */
    const std::string robot_name = "3segment";
    /** @brief mass of each section and connector of entire robot, in kg. The model sets the mass of each PCC element based on this and the estimated volume.
     * segment 2: 160g, 1-2 connector: 20g, segment: 1 82g, gripper: 23g
     * fake value for segment 3
     */
    const std::array<double, 6> masses = {0.19, 0.02, 0.160, 0.020, 0.082, 0.023};
    /** @brief length of each part, in m
     * account for a bit of stretching under pressure...
     * {length of base segment, length of base connector piece, ..., length of tip segment} */
    const std::array<double, 6> lengths = {0.125, 0.02, 0.125, 0.02, 0.125, 0.02};
    /**
     * @brief outer diameters of semicircular chamber
     * {base of base segment, tip of base segment = base of next segment, ...}
     */
    const std::array<double, 4> diameters = {0.042, 0.035, 0.028, 0.0198};
    const int num_segments = 3;
    const int sections_per_segment = 1;
    const int q_size = 2*num_segments*sections_per_segment;
    /** @brief angle of arm rel. to upright */
    const double armAngle = 180;
    const ControllerType controller = ControllerType::lqr;
}

namespace srl{
    /**
     * @brief represents the position \f$q\f$, velocity \f$\dot q\f$, and acceleration \f$\ddot q\f$ for the soft arm.
     */
    class State{
    public:
        Matrix<double, st_params::q_size, 1> q;
        Matrix<double, st_params::q_size, 1> dq;
        Matrix<double, st_params::q_size, 1> ddq;
        State(){
            q.setZero();
            dq.setZero();
            ddq.setZero();
        }
    };
}

/**
 * @brief convert from phi-theta parametrization to longitudinal, for a single PCC section.
 */
inline void phiTheta2longitudinal(double phi, double theta, double& Lx, double& Ly){
    Lx = -cos(phi) * theta;
    Ly = -sin(phi) * theta;
}

/**
 * @brief convert from longitudinal to phi-theta parametrization, for a single PCC section.
 */
inline void longitudinal2phiTheta(double Lx, double Ly, double& phi, double& theta){
    phi = atan2(-Ly, -Lx);
    if (Lx == 0 && Ly == 0)
        phi = 0;
    theta = sqrt(pow(Lx, 2) + pow(Ly, 2));
}
