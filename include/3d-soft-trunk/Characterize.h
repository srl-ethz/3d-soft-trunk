#pragma once

#include "ControllerPCC.h"

class Characterize: public ControllerPCC {
public:
    Characterize(CurvatureCalculator::SensorType sensor_type);

    /** @brief log a graph containing radial intensity of arm's actuation
     * @details the arm doesn't actuate equally in all directions, this is meant to enable counteracting that
     * this takes very long to execute (30 minutes) as it is very precise
     * @param segment segment characterized, starts at 0 (base) */
    void logRadialPressureDist(int segment);

    /** @brief calculate optimal coefficients for gravity vs K term using least squares fitting
     * */
    void calcGravK(int segment, int directions = 8);
    
private:
    const double deg2rad = 0.01745329;
};