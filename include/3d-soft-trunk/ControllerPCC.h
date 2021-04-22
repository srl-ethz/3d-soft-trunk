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

    /** @brief get current kinematic state of the arm */
    void get_state(srl::State &pose);
    /** @brief get current pressure output to the arm */
    void get_pressure(VectorXd &p_vectorized);


private:

    /**
     * actuate the arm using generalized forces
     * @param f generalized force expressed in st_params::parametrization space
     */
    void actuate(VectorXd f);

    /**
     * @brief convert pseudopressures to real pressures
     * WARNING: Does not perform any unit conversion, so give it pseudopressures in mbar (or convert to mbar after 2 -> 3 conversion) 
     * @param pressure_pseudo 2d input pressure
     * @return VectorXd of 3d output pressure
     */
    VectorXd pseudo2real(VectorXd& pressure_pseudo);


    /**
     * @brief give a pressure vector which will compensate for gravity + state related forces (includes velocity based ones)
     * @param state state for which should be equalized
     * @return VectorXd of pressures, unit mbar
     */
    VectorXd gravity_compensate(srl::State state);

    std::unique_ptr<SoftTrunkModel> stm;
    std::unique_ptr<ValveController> vc;
    std::unique_ptr<CurvatureCalculator> cc;

    std::string bendlabs_portname = "/dev/ttyUSB0";

    // parameters for PID controller
    std::array<double, 3> Ku = {3000, 2500, 2000}; /** @brief ultimate gain for each segment, used in Ziegler-Nichols method */
    std::array<double, 3> Tu = {0.7, 0.7, 0.7}; /** @brief oscillation period for each segment, used in Ziegler-Nichols method */
    std::vector<MiniPID> miniPIDs;

    /** @brief baseline pressure of arm. The average of the pressures sent to a segment should be this pressure.
     * for DragonSkin 30, set to 300.
     * for DragonSkin 10, set to 150.
     * (not throughly examined- a larger or smaller value may be better)
     */
    const int p_offset = 50;
    const int p_max = 600; // 400 for DS 10, 1200 for DS 30

    const double dt = 1./30.;

    bool is_initial_ref_received = false;

    // mapping between 2D pseudopressures and 3 chambers
    MatrixXd chambermap = MatrixXd::Zero(2,3);

    std::array<double, 3> segment_weight = {1.2, 1, 1}; //weightings for respective segments in gravity compensate

    std::thread control_thread;
    std::mutex mtx;

    void control_loop();

    // arm configuration
    srl::State state;
    srl::State state_ref;


    VectorXd p = VectorXd::Zero(3 * st_params::num_segments);
};
