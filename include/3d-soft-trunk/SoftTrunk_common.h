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
#include <yaml-cpp/yaml.h>
#include <fmt/core.h>

using namespace Eigen;

enum class ControllerType {
    dynamic,
    pid,        // for PID control, error is directly converted to pressure (i.e. alpha not used)
    gravcomp,   //attempts to hold current position, msubmake arm compliant
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
        /** @brief initialize and set the size of the vectors at the same time. */
        State(const int q_size){
            setSize(q_size);
        }
        /** @brief default constructor, the size of the vectors must be set later with setSize(). */
        State(){
        }
        /** @brief designate size of state (i.e. degrees of freedom) */
        void setSize(const int q_size){
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
 * check apps/example_SoftTrunkModel.cpp to for a demo of how to edit these values
 * @todo the parameters ideally should be private members, and only changeable through functions, to prevent further change after finalize() is called. for now, leave it as is, maybe change when more people start to use it.
 * @todo implement load_yaml function
 */
class SoftTrunkParameters{
public:
    /** @brief return empty state with appropriate size. */
    srl::State getBlankState() const {
        assert(is_finalized());
        srl::State state{q_size};
        return state;
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
    std::vector<double> masses = {0.160, 0.020, 0.140, 0.023};
    /** @brief length of each part, in m
     * account for a bit of stretching under pressure...
     * {length of base segment, length of base connector piece, ..., length of tip segment} */
    std::vector<double> lengths = {0.125, 0.02, 0.125, 0.02};
    /**
     * @brief outer diameters of semicircular chamber
     * {base of base segment, tip of base segment = base of next segment, ...}
     */
    std::vector<double> diameters = {0.035, 0.028, 0.025};
    /** @brief angle of arm rel. to upright */
    double armAngle = 180;

    /** @brief shear modulus of Dragon Skin 10, in Pa
     * literature value for shear modulus is 85000. The values here are determined from characterization_actuation and characterize.py.
     * @todo the value for the base segment is fake now, must run characterization on the real segment
     */
    std::vector<double> shear_modulus = {40686*2.8112137272542075, 59116*2.109658268144279};
    std::vector<double> drag_coef = {28000., 8000.};

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
     * @details the YAML file should be placed inside of the config folder to be read
     * @param filename filename, no need append .yaml as this is done automatically
     */
    void load_yaml(const std::string filename){
        assert(!is_finalized());
        std::string loc = fmt::format("{}/config/{}",SOFTTRUNK_PROJECT_DIR,filename);
        YAML::Node params = YAML::LoadFile(loc);
        this->robot_name = params["robot_name"].as<std::string>();
        this->num_segments = params["num_segments"].as<int>();
        this->sections_per_segment = params["sections_per_segment"].as<int>();
        this->masses = params["masses"].as<std::vector<double>>();
        this->lengths = params["lengths"].as<std::vector<double>>();
        this->diameters = params["diameters"].as<std::vector<double>>();
        this->armAngle = params["armAngle"].as<double>();
        this->shear_modulus = params["shear_modulus"].as<std::vector<double>>();
        this->drag_coef = params["drag_coef"].as<std::vector<double>>();  
    }
    
    bool is_finalized() const {
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
