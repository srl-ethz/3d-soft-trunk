#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include <mobilerack-interface/QualisysClient.h>

class MotionCapture{
public:
    MotionCapture(const SoftTrunkParameters& st_params);

    ~MotionCapture();

    /** @brief get an individual homogenous transformation from qualisys
     * @param id number of the frame to grab
     * @details this function is for debugging purposes */
    Eigen::Transform<double, 3, Eigen::Affine> get_frame(int id);
    
    SoftTrunkParameters st_params_;

    srl::State state_;


private:
 
    /** @brief the connection to QTM server */
    std::unique_ptr<QualisysClient> optiTrackClient;
    /** @brief absolute qualisys tranformations */
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> abs_transforms_;

    unsigned long long int timestamp_ = 0;
    unsigned long long int last_timestamp_ = 0;

    std::thread calculatorThread;
    void calculator_loop();

    std::mutex mtx;
    bool run_ = true;

};