//
// Created by yasu on 26/10/18.
//

#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include "mobilerack-interface/ValveController.h"
#include "3d-soft-trunk/Model.h"
#include "3d-soft-trunk/StateEstimator.h"
#include <mutex>


/**
 * @brief Implements the PCC controller as described in paper.
 * @details Different controllers can be implemented by creating a child class of this class.
 * Includes a simulator functionality, which integrates the analytical model forward in time.
 **/
class ControllerPCC {
public:
    ControllerPCC(const SoftTrunkParameters st_params);

    ~ControllerPCC();

    /** @brief Set the reference state of the arm (configuration space)*/
    void set_ref(const srl::State &state_ref);
    /** @brief Set the reference state of the arm (task space)*/
    void set_ref(const Vector3d &x_ref, const Vector3d &dx_ref = Vector3d::Zero(), const Vector3d &ddx_ref = Vector3d::Zero());


    /** @brief Toggles logging of x,q to a csv file, filename is defined in string filename elsewhere */
    void toggle_log();

    /** @brief Forward simulate the model while inputting pressure p
    *   @return If the simulation was successful (true) or overflowed (false) */
    bool simulate(const VectorXd &p);

    /** @brief Toggles gripper */
    void toggleGripper();

    /** @brief Actuate the arm
     * @param p Pressure vector, 3 pressures per segment for default SoPrA*/
    void actuate(const VectorXd &p);

    bool gripperAttached_ = false;

    /** @brief Load attached to the tip of the arm, in newton*/
    double loadAttached_ = 0.;

    // arm configuration
    srl::State state_;
    Vector3d x_;
    Vector3d dx_;
    Vector3d ddx_;
    srl::State state_prev_; //for simulation

    //references
    Vector3d x_ref_;
    Vector3d dx_ref_;
    Vector3d ddx_ref_;
    srl::State state_ref_;

    DynamicParams dyn_;
    double dt_ = 1./50;

    const SoftTrunkParameters st_params_;

    /** @brief Log filename */
    std::string filename_;


protected:


    /**
     * @brief Determine a pseudopressure which will compensate for gravity + state related forces
     * @details Effectively makes the arm "weightless", i.e. PID control should work
     * @param state state for which should be equalized
     * @return VectorXd of pseudopressures, unit mbar
     */
    VectorXd gravity_compensate(const srl::State state);

    /** @brief Check if J is in a singularity (within a threshold)
    *   @return Order of the singularity
    */
    int singularity(const MatrixXd &J);


    std::unique_ptr<Model> mdl_;
    std::unique_ptr<StateEstimator> ste_;
    std::unique_ptr<ValveController> vc_;

    const int p_max = 700; // 700 for DS 10, 1200 for DS 30

    double t_ = 0;

    bool is_initial_ref_received = false;

    std::thread control_thread_;
    std::thread sensor_thread_;
    std::thread model_thread_;

    void sensor_loop();
    void model_loop();

    std::mutex mtx;

    bool gripping_ = false;

    //actuation vectors, p is pressures and f is torques
    VectorXd p_;
    VectorXd f_;

    //logging variables
    bool logging_ = false;
    unsigned long long int initial_timestamp_;
    std::fstream log_file_;
    void log(double t);

    bool run_;
private:
    /** @brief Forward simulate using Beeman method
    *   @details Explained here https://www.compadre.org/PICUP/resources/Numerical-Integration/ */
    bool Beeman(const VectorXd &p);
};
