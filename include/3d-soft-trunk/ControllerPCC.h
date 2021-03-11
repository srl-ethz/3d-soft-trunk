//
// Created by yasu on 26/10/18.
//

#pragma once

#include "SoftTrunk_common.h"
#include "CurvatureCalculator.h"
#include "SoftTrunkModel.h"
#include <mobilerack-interface/ValveController.h>
#include <MiniPID.h>
#include <mutex>


/**
 * @brief Implements the PCC controller as described in paper.
 * @details It receives pointers to instances of AugmentedRigidArm and SoftArm, so it can access instances of those classes to retrieve information about them that can be used in the Manager class to control the Soft Trunk.
 * By setting USE_PID_CURVATURE_CONTROL to true in SoftTrunk_common_defs.h, it can also do PID control.
 * @todo update to fully use SoftTrunkModel. This should get much shorter.
 */
class ControllerPCC {
public:
    ControllerPCC(CurvatureCalculator::SensorType sensor_type);

    /** @brief set the reference pose (trajectory) of the arm
     */
    void set_ref(const srl::State &pose_ref);

    /** @brief get current kinematic parameters of the arm */
    void get_kinematic(srl::State &pose);
    /** @brief get current pressure output to the arm */
    void get_pressure(VectorXd &p_vectorized);



private:
    std::unique_ptr<SoftTrunkModel> stm;
    std::unique_ptr<ValveController> vc;
    std::unique_ptr<CurvatureCalculator> cc;

    std::string qtm_address = "192.168.0.0";
    std::string bendlabs_portname = "/dev/ttyUSB0";

    // parameters for dynamic controller
    VectorXd K_p; /** @brief P gain for pose FB */
    VectorXd K_d; /** @brief D gain for pose FB */
    double alpha = 2.16e-4; /** @brief used to convert generalized force tau to pressure P (P = tau / alpha). Not used for PID control. Uses model-based value, which is very close to experimental value 2.88e-4. */
    double beta; // ????

    // parameters for PID controller
    std::array<double, 3> Ku = {800, 500, 400}; /** @brief ultimate gain for each segment, used in Ziegler-Nichols method */
    std::array<double, 3> Tu = {0.7, 0.7, 0.7}; /** @brief oscillation period for each segment, used in Ziegler-Nichols method */
    std::vector<MiniPID> miniPIDs;

    /** @brief baseline pressure of arm. The average of the pressures sent to a segment should be this pressure.
     * for DragonSkin 30, set to 300.
     * for DragonSkin 10, set to 150.
     * (not throughly examined- a larger or smaller value may be better)
     */
    const int p_offset = 150;
    const int p_max = 400; // 400 for DS 10, 1200 for DS 30

    bool use_feedforward = false;
    bool simulate = false;

    const double dt = 1./30.;

    bool is_initial_ref_received = false;

    /**
     * actuate the arm using generalized forces
     * @param f generalized force expressed in st_params::parametrization space
     */
    void actuate(VectorXd f);

    std::thread control_thread;
    std::mutex mtx;

    void control_loop();

    // arm configuration
    srl::State state;
    srl::State state_ref;
    VectorXd q_all_ref;


    VectorXd p_vectorized; /** @brief vector that expresses net pressure for X&Y directions, for each segment */

};
