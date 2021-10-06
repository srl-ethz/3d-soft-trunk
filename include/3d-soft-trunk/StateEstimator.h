#pragma once

#include "3d-soft-trunk/Softtrunk_common.h"

class StateEstimator{
public:
    /** @brief construct state estimator */
    StateEstimator(const SoftTrunkParameters& st_params);

    /** @brief get primary state from estimator */
    void get_state(srl::State& state);
    
    /** @brief get all states from estimator, unfiltered */
    void get_states(std::vector<srl::State>& states);


private:
    /** @brief vector containing all active sensors */
    std::vector<EstimatorType> sensors;
    FilterType filter_type;

};