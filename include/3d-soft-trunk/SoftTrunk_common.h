/**
 * @file SoftTrunk_common.h
 * @brief defines various constants and convenience functions for the Soft Trunk that are used across different files.
 */

#pragma once

#include <mobilerack-interface/common.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <thread>
#include <assert.h>
#include <array>

using namespace Eigen;

enum class ControllerType {
    dynamic,
    pid,        // for PID control, error is directly converted to pressure (i.e. alpha not used)
    gravcomp,   //attempts to hold current position, make arm compliant
    lqr,        //LQR controller, directly to pressure
    osc,        //OSC controller, coordinates are task space
};

namespace srl{
    /**
     * @brief represents the position \f$q\f$, velocity \f$\dot q\f$, and acceleration \f$\ddot q\f$ for the soft arm.
     */
    class State{
    public:
        VectorXd q;
        VectorXd dq;
        VectorXd ddq;
        /** @brief designate size of state (i.e. degrees of freedom) */
        State(const int q_size){
            q = VectorXd::Zero(q_size);
            dq = VectorXd::Zero(q_size);
            ddq = VectorXd::Zero(q_size);
        }
    };
}

/**
 * @brief save various parameters related to the configuration of the soft trunk.
 * The parameters are first populated by their default values. After customizing them by changing the member variables or reading from a YAML file,
 * call finalize() to run sanity checks on the values, and to set other parameters.
 * @todo the parameters ideally should be private members, and only changeable through functions, to prevent further change after finalize() is called. for now, leave it as is, maybe change when more people start to use it.
 * @todo implement load_yaml function
 */
class SoftTrunkParameters{
public:
    /** @brief return empty state with appropriate size. */
    srl::State empty_state(){
        assert(is_finalized());
        return srl::State(q_size);
    }
    /** @brief name of robot (and of urdf / xacro file) */
    std::string robot_name = "2segment";
    /** @brief number of actuated segments */
    int num_segments = 2;
    /** @brief number of PCC elements per section */
    int sections_per_segment = 1;
    /** @brief mass of each section and connector of entire robot, in kg. The model sets the mass of each PCC element based on this and the estimated volume.
     * segment 2: 160g, 1-2 connector: 20g, segment: 1 82g, gripper: 23g
     */
    std::array<double, 4> masses = {0.160, 0.020, 0.082, 0.023};
    /** @brief length of each part, in m
     * account for a bit of stretching under pressure...
     * {length of base segment, length of base connector piece, ..., length of tip segment} */
    std::array<double, 4> lengths = {0.125, 0.02, 0.125, 0.02};
    /**
     * @brief outer diameters of semicircular chamber
     * {base of base segment, tip of base segment = base of next segment, ...}
     */
    std::array<double, 3> diameters = {0.035, 0.028, 0.0198};
    /** @brief angle of arm rel. to upright */
    double armAngle = 180;

    /** @brief shear modulus of Dragon Skin 10, in Pa
     * literature value for shear modulus is 85000. The values here are determined from characterization_actuation and characterize.py.
     * @todo the value for the base segment is fake now, must run characterization on the real segment
     */
    std::array<double, 2> shear_modulus = {34200., 56500.};
    std::array<double, 2> drag_coef = {28000., 8000.};

    /** @brief degrees of freedom of arm. is set when finalize() is called */
    int q_size;

    void finalize(){
        assert(!is_finalized()); // already finalized

        // run sanity checks to make sure that at least the size of the arrays make sense
        assert(num_segments * 2 == masses.size());
        assert(num_segments * 2 == lengths.size());
        assert(num_segments + 1 == diameters.size());
        assert(num_segments == shear_modulus.size());
        assert(num_segments == drag_coef.size());

        q_size = 2*num_segments*sections_per_segment;
        finalized = true;
    }

    /** @brief populate the parameters by reading from a YAML file
     * @todo implement this
     */
    void load_yaml(const std::string filename){
        assert(!is_finalized());
    }
    
    bool is_finalized(){
        return finalized;
    }
private:
    bool finalized = false;
};




/**
 * @brief convert from phi-theta parametrization to longitudinal, for a single PCC section.
 */
inline void phiTheta2longitudinal(double phi, double theta, double& Lx, double& Ly){
    Lx = -cos(phi) * theta;
    Ly = -sin(phi) * theta;
}

/**
 * @brief convert from longitudinal to phi-theta parametrization, for a single PCC section.
 */
inline void longitudinal2phiTheta(double Lx, double Ly, double& phi, double& theta){
    phi = atan2(-Ly, -Lx);
    if (Lx == 0 && Ly == 0)
        phi = 0;
    theta = sqrt(pow(Lx, 2) + pow(Ly, 2));
}
