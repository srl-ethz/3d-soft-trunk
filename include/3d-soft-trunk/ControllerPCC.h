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

    /** @brief set the reference pose (trajectory) of the arm
     */
    void set_ref(const srl::State &state_ref);
    void set_ref(const Vector3d &x_ref, const Vector3d &dx_ref = Vector3d::Zero(), const Vector3d &ddx_ref = Vector3d::Zero());


    /** @brief toggles logging of x,q to a csv file, filename is defined in string filename elsewhere */
    void toggle_log();

    /** @brief forward simulate the stm by dt while inputting pressure p
    *   @return if the simulation was successful (true) or overflowed (false) */
    bool simulate(const VectorXd &p);

    /** @brief toggles gripper */
    void toggleGripper();

    /** @brief gripper attached */
    bool gripperAttached_ = false;

    /** @brief load attached to the arm, in Newtons */
    double loadAttached_ = 0.;

    // arm configuration
    srl::State state_;

    srl::State state_prev_; //for simulation

    //references
    Vector3d x_ref_;
    Vector3d dx_ref_;
    Vector3d ddx_ref_;
    srl::State state_ref_;

    DynamicParams dyn_;

    const SoftTrunkParameters st_params_;

    /** @brief log filename */
    std::string filename_;


protected:

    /**
     * actuate the arm using generalized forces
     * @param p pressure vector, 3 pressures per segment
     */
    void actuate(const VectorXd &p);

    /**
     * @brief give a pseudopressure vector which will compensate for gravity + state related forces (includes velocity based ones)
     * @details gravity_compensate3 uses the stm->A matrix instead of the stm->A_pseudo, should functionally be the same
     * @param state state for which should be equalized
     * @return VectorXd of pseudopressures, unit mbar
     */
    VectorXd gravity_compensate(const srl::State state);

    /** @brief check if J is in a singularity
    *   @return order of the singularity
    */
    int singularity(const MatrixXd &J);


    std::unique_ptr<Model> mdl_;
    std::unique_ptr<StateEstimator> ste_;
    std::unique_ptr<ValveController> vc_;

    const int p_max = 700; // 700 for DS 10, 1200 for DS 30

    double dt_ = 1./50;
    double t_ = 0;

    bool is_initial_ref_received = false;

    std::thread control_thread_;
    std::thread sensor_thread_;
    std::thread model_thread_;

    void control_loop();
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
    /** @brief forward simulate using beeman method
    *   @details explained here https://www.compadre.org/PICUP/resources/Numerical-Integration/ */
    bool Beeman(const VectorXd &p);
};
