#pragma once

#include "ControllerPCC.h"
#include <random>

class Characterize: public ControllerPCC {
public:
    Characterize(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type);

    /** @brief log a graph containing radial intensity of arm's actuation
     * @details the arm doesn't actuate equally in all directions, this is meant to enable counteracting that
     * this takes very long to execute (30 minutes) as it is very precise
     * @param segment segment characterized, starts at 0 (base) */
    void logRadialPressureDist(int segment, std::string filename);

    /** @brief calculate optimal coefficients for gravity vs K term using least squares fitting
     * */
    void calcK(int segment, int directions = 8, int verticalsteps = 5, std::string fname = "K_log");
    
    /** @brief monte carlo task space analysis
     * @details approach many random pressure points as to analyse the EE location 
     * @param points number of randomized points to approach
     * @param speed factor for speed, default 1.0
     * @param dt delta used for steps during transitionary behavior*/
    void taskSpaceAnalysis(int points = 15, double speed = 1.0, double dt = 0.05);

private:
    const double deg2rad = 0.01745329;
};