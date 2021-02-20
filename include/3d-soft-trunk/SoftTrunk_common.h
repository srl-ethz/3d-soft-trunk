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

/** @brief placement of arm */
enum class ArmConfigurationType {
    stalactite /** hanging from ceiling- stick "tight" to the ceiling */,
    stalagmite /** placed on floor- rise "might"ily from the floor */
};
enum class ControllerType {
    dynamic,
    pid /** for PID control, error is directly converted to pressure (i.e. alpha not used) */
};

/** @brief parameters that define the configuration of the Soft Trunk */
namespace st_params {
    /** @brief name of robot (and of urdf / xacro file), make it different for differently configured robots */
    const std::string robot_name = "2segment";
    /** @brief mass of entire robot, in kg */
    double totalMass = 0.3;
    /** @brief length of each part, in m 
     * {length of base segment, length of base connector piece, ..., length of tip segment} */
    std::array<double, 3> lengths = {0.12, 0.02, 0.12};
    /**
     * @brief outer diameters of semicircular chamber
     * {base of base segment, tip of base segment = base of next segment, ...}
     */
    std::array<double, 3> diameters = {0.032, 0.028, 0.02};
    const int num_segments = 2;
    const int sections_per_segment = 3;

    const ArmConfigurationType armConfiguration = ArmConfigurationType::stalactite;
    const ControllerType controller = ControllerType::pid;
}

/**
 * @brief convert from phi-theta parametrization to longitudinal, for a single PCC section.
 */
void phiTheta2longitudinal(double phi, double theta, double& Lx, double& Ly){
    Lx = -cos(phi) * theta;
    Ly = -sin(phi) * theta;
}

/**
 * @brief convert from longitudinal to phi-theta parametrization, for a single PCC section.
 */
void longitudinal2phiTheta(double Lx, double Ly, double& phi, double& theta){
    phi = atan2(Ly, Lx);
    if (Lx == 0 && Ly == 0)
        phi = 0;
    theta = sqrt(pow(Lx, 2) + pow(Ly, 2));
}