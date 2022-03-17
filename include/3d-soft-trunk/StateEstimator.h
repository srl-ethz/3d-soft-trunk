#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include "3d-soft-trunk/Sensors/BendLabs.h"
#include "3d-soft-trunk/Sensors/MotionCapture.h"

/** @brief The StateEstimator object polls any number of sensors to obtain states. It also has functionality to filter states, although so far no filters have been implemented*/
class StateEstimator{
public:
    StateEstimator(const SoftTrunkParameters& st_params);
    ~StateEstimator();


    /** @brief Fetch new state data from all sensors, and filter them. */
    void poll_sensors();

    /** @brief The clean, filtered state */
    srl::State state_;
    /** @brief Raw sensor data from all sensors. Unfiltered. */
    std::vector<srl::State> all_states_;

    const SoftTrunkParameters st_params_;
private:

    /** @brief vector containing all active sensors */
    std::vector<SensorType> sensors_;

    std::unique_ptr<MotionCapture> mocap_;
    std::unique_ptr<BendLabs> bendlabs_;

    /** @brief Grab newest states of sensors */
    void get_states();

    /** @brief Update the filtered state according to filter*/
    void get_filtered_state();

    FilterType filter_type_ = FilterType::none;

    std::mutex mtx;

};