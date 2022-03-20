#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include <mobilerack-interface/SerialInterface.h>

/** @brief BendLabs sensor reader
 * @details Using following sensor: https://www.bendlabs.com/products/2-axis-soft-flex-sensor/
 * @details This sensor can only read curvature, tip positions must be estimated with forward kinematics */
class BendLabs{
public:

    BendLabs(const SoftTrunkParameters& st_params);

    ~BendLabs();

    srl::State state_;
    
private:
    SoftTrunkParameters st_params_;

    srl::State state_prev_;

    std::unique_ptr<SerialInterface> serialInterface;

    std::vector<float> bendLab_data_;
    std::vector<float> bendLab_data_prev_;

    /** @brief timestamp tracked in us */
    unsigned long long int timestamp_ = 0;
    unsigned long long int last_timestamp_ = 0;

    std::thread calculatorThread;
    void calculator_loop();

    std::mutex mtx;
    bool run = true;
};