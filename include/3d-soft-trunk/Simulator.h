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
     * @param dt time of control input step, p is regarded as constant for this time
     * @param steps resolution of integration, higher value is more computationally expensive but delivers more accurate results
     */

    Simulator(SoftTrunkModel &stm, Simulator::SimType sim_type, const double &dt, const int &steps);


    /**
     * @brief simulate a control step
     * @param p pressure input vector, size 3*st_params::num_segments. viewed as constant for the input time
     * @param pose pose which is forward-simulated
     * returns true if simulation succeeds
     */
    bool simulate(const VectorXd &p, Pose &pose);


    /**
     * @brief starts logging pose and time data for every control step
     * @param filename filename of output csv, placed in 3d-soft-trunk directory. {filename}.csv
     */
    void start_log(std::string filename);

    void end_log();


private:
    /**
     * @brief numerically forward-integrate using beeman approximation
     * @param simtime time over which is integrated
     * returns true if integration delivers useable value
     */
    bool Beeman(const VectorXd &p, Pose &pose);

    /**
     * @brief numerically forward-integrate using euler-richardson approximation
     * @param simtime time over which is integrated
     * returns true if integration delivers useable value
     */
    bool Euler(const VectorXd &p, Pose &pose);



    Pose pose_prev; //for Beeman
    Pose pose_mid;  //For Euler-Richardson


    SoftTrunkModel &stm;
    SimType sim_type;
    const int &steps;
    const double &dt;
    const double simtime;
    double t;

    std::fstream log_file;
    std::string filename;
    bool logging;
};
