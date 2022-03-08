#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include "3d-soft-trunk/Sensors/BendLabs.h"
#include "3d-soft-trunk/Sensors/MotionCapture.h"

class StateEstimator{
public:
    /** @brief construct state estimator */
    StateEstimator(const SoftTrunkParameters& st_params);

    ~StateEstimator();

    /** @brief have the state estimator update all states from sensors */
    void poll_sensors();

    /** @brief get n'th object */
    Eigen::Transform<double, 3, Eigen::Affine> get_object(int id);

    srl::State state;

    const SoftTrunkParameters st_params;
private:



    std::vector<srl::State> states_;

    /** @brief vector containing all active sensors */
    std::vector<SensorType> sensors;
    std::unique_ptr<MotionCapture> mocap;
    std::unique_ptr<BendLabs> bendlabs;

    /** @brief gets state of i'th sensor */
    void get_state_from_ptr(srl::State& state, int i);

    /** @brief returns filtered state */
    srl::State get_filtered_state();

    FilterType filter_type;

    std::mutex mtx;


};