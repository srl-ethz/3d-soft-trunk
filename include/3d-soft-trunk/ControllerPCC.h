//
// Created by yasu on 26/10/18.
//

#pragma once

#include "SoftTrunk_common.h"
#include "CurvatureCalculator.h"
#include "SoftTrunkModel.h"
#include <mobilerack-interface/ValveController.h>
#include <mutex>


/**
 * @brief Implements the PCC controller as described in paper.
 * @details Different controllers can be implemented by creating a child class of this class.
 * Includes a simulator functionality, which integrates the analytical model forward in time.
 **/
class ControllerPCC {
public:
    ControllerPCC(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects = 0);

    /** @brief set the reference pose (trajectory) of the arm
     */
    void set_ref(const srl::State &state_ref);
    void set_ref(const Vector3d x_ref, const Vector3d &dx_ref = Vector3d::Zero(), const Vector3d &ddx_ref = Vector3d::Zero());

    /** @brief get current kinematic state of the arm */
    void get_state(srl::State &state);

    /** @brief get the tip x coordinates */
    void get_x(Vector3d &x);

    /** @brief set position of arm, only use for simulations! */
    void set_state(const srl::State &state);

    /** @brief get current pressure output to the arm */
    void get_pressure(VectorXd &p_vectorized);

    /** @brief return x_tip */
    Vector3d get_x();

    /**
     * @brief toggles logging of x,q to a csv file, filename is defined in string filename elsewhere
     */
    void toggle_log();

    /** @brief change log filename to string */
    void set_log_filename(const std::string s);

    /**
    *@brief return segment tip transformation
    */
    Eigen::Transform<double, 3, Eigen::Affine> get_H(int segment_id);

    /**
     * @brief return transformations of objects in qualisys (objects =/= soft arm)
     */
    std::vector<Eigen::Vector3d> get_objects();

    /** @brief forward simulate the stm by dt while inputting pressure p
    *   @return if the simulation was successful (true) or overflowed (false) */
    bool simulate(const VectorXd &p);

    /** @brief new chamber configuration */
    void newChamberConfig(Vector3d &angles);

    /** @brief sets the frequency of the simulator */
    void set_frequency(const double hz);

    /** @brief toggles gripper */
    void toggleGripper();
    /** @brief gripper attached */
    bool gripperAttached = false;
    double loadAttached = 0.;

protected:

    const SoftTrunkParameters st_params;
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
    VectorXd gravity_compensate3(const srl::State state);

    /** @brief check if J is in a singularity
    *   @return order of the singularity is in, also acts as bool
    */
    int singularity(const MatrixXd &J);


    std::unique_ptr<ValveController> vc;
    std::unique_ptr<SoftTrunkModel> stm;
    std::unique_ptr<CurvatureCalculator> cc;
    


    std::string bendlabs_portname = "/dev/ttyUSB0";


    /** @brief baseline pressure of arm. The average of the pressures sent to a segment should be this pressure.
     * for DragonSkin 30, set to 300.
     * for DragonSkin 10, set to 150.
     * (not throughly examined- a larger or smaller value may be better)
     */
    const int p_offset = 0;
    const int p_max = 700; // 400 for DS 10, 1200 for DS 30

    

    double dt = 1./30.;
    double t;

    bool is_initial_ref_received = false;

    std::thread control_thread;
    std::mutex mtx;

    void control_loop();

    // arm configuration+target positions
    srl::State state;
    srl::State state_ref;
    srl::State state_prev; //for simulation
    
    Vector3d x;
    Vector3d x_ref;
    Vector3d dx;
    Vector3d dx_ref;
    Vector3d ddx_ref;

    bool gripping = false;

    //actuation vectors, p is pressures and f is torques
    VectorXd p;
    VectorXd f;

    //logging variables
    bool logging = false;
    unsigned long long int initial_timestamp;
    std::fstream log_file;
    std::string filename;
    void log(double t);

    //qualisys variables
    Eigen::Transform<double, 3, Eigen::Affine> base_transform;
    int objects;
    CurvatureCalculator::SensorType sensor_type;

    //parameters with sopra attached druing lifting
    double p1 = 2.27286685805334e+16; 
    double p2 =  -1.44742813860741e+16;
    double p3 =  5.26144455255893e+15;
    double p4 =   -1.19403849976615e+15;
    double p5 =   173236603400900;
    double p6 =   -15691774818037.8;
    double p7 =   811336068389.078;
    double p8 =   -18333497313.1710;
    double p9 =  -1.55973266940115e+16;

    //parameters with sopra during falling

    double pa1 = -3.46533160247876e+15; 
    double pa2 =  2.25018172335107e+15;
    double pa3 =  -834656736243444;
    double pa4 =   193435836054681;
    double pa5 =   -28681818851200.6;
    double pa6 =   2657163630691.87;
    double pa7 =   -140622269201.993;
    double pa8 =   3254846129.69969;
    double pa9 =  2.33400234637593e+15;


private:
    /** @brief forward simulate using beeman method
    *   @details explained here https://www.compadre.org/PICUP/resources/Numerical-Integration/
    */
    bool Beeman(const VectorXd &p);

};
