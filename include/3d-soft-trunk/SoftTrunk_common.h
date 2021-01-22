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

/** @brief which type of parametrization is used to describe the PCC configuration. */
enum class ParametrizationType {
    phi_theta /** as used in Katzschmann2019 */,
    longitudinal /** see [yasu's report](https://www.overleaf.com/read/rsnsrjphmpxx) & DellaSantina2020. longitudinal length calcuated with unit radius */
};
/** @brief the link structure of the rigid model */
enum class RigidModelType {
    original /** as proposed in Katzschmann2019 */,
    straw_bend /** see yasu's report */
};
/** @brief placement of arm */
enum class ArmConfigurationType {
    stalactite /** hanging from ceiling- stick "tight" to the ceiling */,
    stalagmite /** placed on floor- rise "might"ily from the floor */
};
enum class ControllerType {
    dynamic,
    pid /** for PID control, error is directly converted to pressure (i.e. alpha not used) */
};

/** @brief robot-specific parameters SoftTrunk*/
namespace st_params {
    const std::string robot_name = "3segment_4chamber";
    /** @brief mass of each segment, in kg */
    std::array<double, 3> masses = {0.12, 0.12, 0.12};
    /** @brief length of each segment, in m */
    std::array<double, 3> lengths = {0.11, 0.11, 0.11};
    const int num_segments = 3;

    const ParametrizationType parametrization = ParametrizationType::longitudinal;
    const RigidModelType rigidModel = RigidModelType::straw_bend;
    const ArmConfigurationType armConfiguration = ArmConfigurationType::stalactite;
    const ControllerType controller = ControllerType::pid;
}