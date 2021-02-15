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
    const int sections_per_segment = 5;

    const ArmConfigurationType armConfiguration = ArmConfigurationType::stalactite;
    const ControllerType controller = ControllerType::pid;
}