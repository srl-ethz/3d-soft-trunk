#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include <mobilerack-interface/QualisysClient.h>

class MotionCapture{
public:
    MotionCapture(const SoftTrunkParameters& st_params);

    ~MotionCapture();

    /** @brief fetch state from qualisys loop */
    void get_state(srl::State& state);

    /** @brief get an individual homogenous transformation from qualisys */
    Eigen::Transform<double, 3, Eigen::Affine> get_frame(int id);

    /** @brief get homogenous transformation corresponding to object 
     * @param id number of tracked object */
    Eigen::Transform<double, 3, Eigen::Affine> get_object(int id);

    void get_x(Vector3d& x);
private:
    SoftTrunkParameters st_params_;

    srl::State state_;

    /** @brief the connection to QTM server */
    std::unique_ptr<QualisysClient> optiTrackClient;
    /** @brief absolute qualisys tranformations */
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> abs_transforms_;

    unsigned long long int timestamp_ = 0;

    std::thread calculatorThread;
    void calculator_loop();
    std::mutex mtx;
    bool run = true;
    unsigned long long int last_timestamp_;
};