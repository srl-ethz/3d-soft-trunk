#pragma once

#include "SoftTrunk_common.h"
#include "SoftTrunkModel.h"
#include <fstream>

/**
 *@brief simulate an existing SoftTrunkModel with input stream of pressures
 * Methods taken from here: https://www.compadre.org/PICUP/resources/Numerical-Integration/
 */
class Simulator{
public:


    /**
     * @brief construct the simulator
     * @param softtrunk passed SoftTrunkModel, the Simulator uses this to calculate approximation
     * @param contrl_step time of control input step, p is regarded as constant for this time
     * @param steps resolution of integration, higher value is more computationally expensive but delivers more accurate results. use value higher than 1 if you want simulation resolution to be higher than control step input
     * @param initialState initial state of the simulation.
     * @details control_step as low as ~0.007 with steps=1 will run at realtime
     */

    Simulator(SoftTrunkModel &stm, double control_step, int steps, const srl::State& initialState);


    /**
     * @brief simulate a control step
     * @param p pressure input vector, size 3*st_params::num_segments. viewed as constant for the input time
     * raise std::runtimeerror if simulation overflows
     */
    void simulate(const VectorXd &p);


    /**
     * @brief starts logging state and time data for every control step
     * @param filename filename of output csv, placed in 3d-soft-trunk directory. {filename}.csv
     */
    void start_log(std::string filename);

    void end_log();


private:
    /**
     * @brief numerically forward-integrate using beeman approximation
     * @param p input pressure
     * returns true if integration delivers useable value
     */
    bool Beeman(const VectorXd &p);
    srl::State state;
    srl::State state_prev; //for Beeman

    SoftTrunkModel &stm;
    int steps;
    const double control_step;
    const double sim_step;
    double t;

    std::fstream log_file;
    std::string filename;
    bool logging;
};
