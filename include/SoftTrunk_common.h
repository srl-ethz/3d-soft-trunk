/**
 * @file SoftTrunk_common.h
 * @brief defines various constants and convenience functions for the Soft Trunk that are used across different files.
 */

#pragma once

#include "mobilerack-interface/common.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <thread>
#include <assert.h>
#include <array>

using namespace Eigen;

/** @brief which type of parametrization is used to describe the PCC configuration. */
enum class ParametrizationType {
    phi_theta, /** as used in Katzschmann2019 */
    longitudinal /** see yasu's report & DellaSantina2020 */
};
/** @brief the link structure of the rigid model */
enum class RigidModelType {
    original, /** as proposed in Katzschmann2019 */
    straw_bend /** see yasu's report */
};
/** @brief placement of arm */
enum class ArmConfigurationType {
    stalactite, /** hanging from ceiling- stick "tight" to the ceiling */
    stalagmite /** placed on floor- rise "might"ily from the floor */
};
enum class ControllerType {
    dynamic,
    pid
};

/** @brief robot-specific parameters SoftTrunk*/
namespace st_params {
    const std::string robot_name = "3segment_4chamber";
    /** @brief mass of each segment, in kg */
    std::array<double, 3> masses = {0.12, 0.12, 0.12};
    /** @brief length of each segment, in m */
    std::array<double, 3> lengths = {0.11, 0.11, 0.11};
    const int num_segments = 1;

    const std::string local_address = "192.168.1.111";

    /** @brief baseline pressure of arm. The average of the pressures sent to a segment should be this pressure.
     * for DragonSkin 30, set to 300.
     * for DragonSkin 10, set to 150.
     * (not throughly examined- a larger or smaller value may be better)
    */
    const int p_offset = 150;

    /** @brief radius of the soft trunk, in meters. */
    const double r_trunk = 0.03;

    // @todo separate into separate namespace for each parametrization
    std::array<double, 3> k = {1, 1, 1};
    std::array<double, 3> beta = {1, 1, 1};
    std::array<double, 6> k_p = {1, 1, 1, 1, 1, 1}; /** @brief P gain for pose FB */
    std::array<double, 6> k_d = {1, 1, 1, 1, 1, 1}; /** @brief D gain for pose FB */
    std::array<double, 6> pid_p = {700, 300, 0, 0, 0, 0};

    /** @brief qualisys-related parameters */
    namespace qualisys {
        const bool log = true; /** @brief log curvature values */
    }

    const ParametrizationType parametrization = ParametrizationType::phi_theta;
    const RigidModelType rigidModel = RigidModelType::straw_bend;
    const ArmConfigurationType armConfiguration = ArmConfigurationType::stalactite;
    const ControllerType controller = ControllerType::dynamic;
}