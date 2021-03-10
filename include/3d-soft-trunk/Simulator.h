#pragma once

#include "SoftTrunk_common.h"
#include "SoftTrunkModel.h"
#include <fstream>

/**
 *@brief simulate an existing SoftTrunkModel based on inputs
 * Methods taken from here: https://www.compadre.org/PICUP/resources/Numerical-Integration/
 */
class Simulator{
public:

    enum class SimType{
        euler,
        beeman
    };
    
    /**
     * @brief construct the simulator
     * @param softtrunk passed SoftTrunkModel, the Simulator uses this to calculate approximation
     * @param sim_type type of approximation to use, currently either the Euler-Richardson (SimType::euler) or the Beeman (SimType::beeman) approximation
     * @param control_step length of control input step in seconds, pressure input p is regarded as constant for this time
     * @param steps resolution of integration, higher value is more computationally expensive but delivers more accurate results. Simulation timestep is control_step/steps.
     */

    Simulator(SoftTrunkModel &stm, Simulator::SimType sim_type, const double &control_step, const int &steps);


    /**
     * @brief simulate a control step
     * @param p pressure input vector, size 3*st_params::num_segments. viewed as constant for the input time
     * @param state state which is forward-simulated
     * @returns true if simulation succeeds
     */
    bool simulate(const VectorXd &p, srl::State &state);


    /**
     * @brief starts logging state and time data for every control step
     * @param filename filename of output csv, placed in 3d-soft-trunk directory. {filename}.csv
     */
    void start_log(std::string filename);

    void end_log();


private:
    /**
     * @brief numerically forward-integrate using beeman approximation
     * @returns true if integration delivers useable value
     */
    bool Beeman(const VectorXd &p, srl::State &state);

    /**
     * @brief numerically forward-integrate using euler-richardson approximation
     * @returns true if integration delivers useable value
     */
    bool Euler(const VectorXd &p, srl::State &state);



    srl::State state_prev; //for Beeman
    srl::State state_mid;  //For Euler-Richardson


    SoftTrunkModel &stm;
    SimType sim_type;
    const int &steps;
    const double &control_step;
    const double sim_step;
    double t;

    std::fstream log_file;
    std::string filename;
    bool logging;
};
