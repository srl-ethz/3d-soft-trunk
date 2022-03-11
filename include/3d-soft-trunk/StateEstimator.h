#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include "3d-soft-trunk/Sensors/BendLabs.h"
#include "3d-soft-trunk/Sensors/MotionCapture.h"

class StateEstimator{
public:
    /** @brief construct state estimator */
    StateEstimator(const SoftTrunkParameters& st_params);

    /** @brief have the state estimator update all states from sensors */
    void poll_sensors();

    /** @brief the "clean", filtered state */
    srl::State state_;

    /** @brief states measured by all sensors */
    std::vector<srl::State> all_states_;

    const SoftTrunkParameters st_params_;
private:

    /** @brief vector containing all active sensors */
    std::vector<SensorType> sensors_;
    std::unique_ptr<MotionCapture> mocap_;
    std::unique_ptr<BendLabs> bendlabs_;

    /** @brief gets state of i'th sensor */
    void get_states();

    /** @brief returns filtered state */
    void get_filtered_state();

    FilterType filter_type_ = FilterType::none;

    std::mutex mtx;

};