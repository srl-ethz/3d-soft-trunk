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
     * @param softtrunk passed SoftTrunkModel, the Simulator uses this to calculate approximations
     * @param pose_passed initial pose given to the Simulator, the variable passed to the Simulator is updated as reference, no returning needed
     * @param sim_type type of approximation to use, currently either the Euler-Richardson (SimType::euler) or the Beeman (SimType::beeman) approximation
     */

    Simulator(SoftTrunkModel &softtrunk, Pose &pose_passed, Simulator::SimType sim_type);


    /**
     * @brief simulate a control step
     * @param p pressure input vector, size 3*st_params::num_segments. viewed as constant for the input time
     * @param dt control step time
     * @param steps resolution of simulation within the step, simulation will perform this amount of numerical integrations. minimum 1
     */
    bool simulate(const VectorXd &p, const double &dt, const int &steps);


    /**
     * @brief starts logging pose and time data for every control step
     * @param filename filename of output csv, placed in 3d-soft-trunk directory. {filename}.csv
     * 
     */
    void start_log(std::string filename);

    void end_log();


private:
    /**
     * @brief numerically forward-integrate using beeman approximation
     * @param simtime time over which is integrated
     */
    bool Beeman(const VectorXd &p, const double &simtime);

    /**
     * @brief numerically forward-integrate using euler-richardson approximation
     * @param simtime time over which is integrated
     */
    bool Euler(const VectorXd &p, const double &simtime);



    Pose &pose;
    Pose pose_prev; //for Beeman
    Pose pose_mid;  //For Euler-Richardson

    SoftTrunkModel &stm;
    SimType sim_type;
    double t;

    std::fstream log_file;
    std::string filename;
    bool logging;
};
