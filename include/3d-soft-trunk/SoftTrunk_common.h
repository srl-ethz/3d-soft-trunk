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
#include <mutex>
#include <cmath>
#include <fstream>


using namespace Eigen;
enum class ControllerType {
    /** @brief as described in katzschmann2019dynamic */
    dynamic,
    /** @brief PID curvature controller */
    pid, 
    /** @brief compensates for gravity, does not accept reference positions */
    gravcomp,
    /** @brief LQR curvature controller */
    lqr,
    /** @brief operation space controller in task space */
    osc,    
};


enum class ModelType {
    /** @brief creates augmented rigid arm equivalent of soft body in drake*/
    augmentedrigidarm, 
    /** @brief uses lagrangian dynamics to derive equation of motion */
    lagrange,
};

enum class CoordType {
    /** @brief thetax,thetay parametrization as proposed in toshimitsu2021sopra */
    thetax,
    /** @brief classic PCC theta phi config */
    phitheta,
};

enum class SensorType {
    /** @brief qualisys motion capture system, needs QTM server */
    qualisys,
    /** @brief bendlabs bend sensor, needs arduino */
    bendlabs,
    /** @brief simulate to obtain states */
    simulator,
};

enum class FilterType {
    none,
};

namespace srl{
    /**
     * @brief represents the position \f$q\f$, velocity \f$\dot q\f$, and acceleration \f$\ddot q\f$ for the soft arm.
     */
    class State{
    public:
        /** @brief joint configuration */
        VectorXd q;
        /** @brief joint velocity */
        VectorXd dq;
        /** @brief joint acceleration */
        VectorXd ddq;
        /** @brief tip transformations relative to base segment */
        std::vector<Eigen::Transform<double, 3, Eigen::Affine>> tip_transforms;
        /** @brief object transformations relative to base segment */
        std::vector<Eigen::Transform<double, 3, Eigen::Affine>> objects;

        CoordType coordtype;
        unsigned long long int timestamp;

        /** @brief initialize and set the size of the vectors at the same time. */
        State(CoordType coordtype, const int q_size) : coordtype(coordtype){
            setSize(q_size);
        }
        /** @brief default constructor, the size of the vectors must be set later with setSize(). */
        State(CoordType coordtype) : coordtype(coordtype){
        }
        /** @brief constructor without coordtype, USE AT OWN RISK, THIS MAY CAUSE ISSUES WITH SAFETY CHECKS */
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

/** @brief Parameters used in the dynamic equation of the arm */
struct DynamicParams{
    /** @brief Coordinate parametrization of the instance */
    CoordType coordtype;
    /** @brief Actuation matrix, maps pressures to torque */
    MatrixXd A;
    /** @brief Actuation matrix simplified to 2 coordinates (x,y "pseudopressures") */
    MatrixXd A_pseudo;
    /** @brief Inertia matrix, maps 2nd derivative of coordinate to torque */
    MatrixXd B;
    /** @brief Vector containing coriolis and related torques*/
    VectorXd c;
    /** @brief Damping matrix, maps 1st derivative of coordinate to torque*/
    MatrixXd D;
    /** @brief Gravity vector, contains gravity torques*/
    VectorXd g;
    /** @brief Stiffness matrix, maps coordinate to torque */
    MatrixXd K;
    /** @brief Coriolis matrix, maps 1st derivative of coordinate to torque */
    MatrixXd S;
    /** @brief Vector containing jacobians of all segment tips */
    std::vector<MatrixXd> J;
    /** @brief Vector containing jacobian derivatives of all segment tips */
    std::vector<MatrixXd> dJ;
    /** @brief Vector containing forward kinematic positions of all segment tips */
    std::vector<Vector3d> x;
};

/**
 * @brief Save various parameters related to the configuration of the soft trunk.
 * The parameters are first populated by their default values. After customizing them by changing the member variables or reading from a YAML file,
 * call finalize() to run sanity checks on the values, and to set other parameters.
 * @todo the parameters ideally should be private members, and only changeable through functions, to prevent further change after finalize() is called. for now, leave it as is, maybe change when more people start to use it.
 */
class SoftTrunkParameters{
public:
    /** @brief return empty state with appropriate size. */
    srl::State getBlankState() const {
        assert(is_finalized());
        srl::State state{coord_type, q_size};
        state.objects.resize(objects);
        state.tip_transforms.resize(num_segments+1+prismatic);
        return state;
    }
    /** @brief Name of robot (and of urdf / xacro file) */
    std::string robot_name = "2segment";

    /** @brief Number of actuated segments */
    int num_segments = 2;

    /** @brief Number of PCC elements per section */
    int sections_per_segment = 1;

    /** @brief Mass of each section and connector of entire robot, in kg. The model sets the mass of each PCC element based on this and the estimated volume.
     * @details {base segment, base connector, ..., tip segment, gripper}*/
    std::vector<double> masses = {0.160, 0.020, 0.082, 0.023};

    /** @brief Length of each part, in m
     * @details {base segment, base connector piece, ..., tip segment} */
    std::vector<double> lengths = {0.125, 0.02, 0.125, 0.02};

    /** @brief Outer diameters of semicircular chamber
     * @details {base of base segment, tip of base segment = base of next segment, ...}*/
    std::vector<double> diameters = {0.035, 0.028, 0.0198};

    /** @brief Angle of arm rel. to upright */
    double armAngle = 180;

    /** @brief Shear modulus of segment silicone, in Pa
    * literature value for shear modulus is 85000. The values here are determined from characterization_actuation and characterize.py.*/
    std::vector<double> shear_modulus = {40686, 59116};

    /** @brief Drag coefficients of arm (air resistance, internal damping...). This is heuristic at the moment. */
    std::vector<double> drag_coef = {28000., 8000.};

    /** @brief Maps valves to the manipulator chambers 
     * scheme: chamber facing x direction (towards user), chamber back left, chamber back right
     * starts at top segment, iterates downwards
     * final chamber is for the gripper
    */
    std::vector<int> valvemap = {3,1,2,4,6,5,0};

    /** @brief Model used to derive the parameters of the dynamic equation */
    ModelType model_type = ModelType::augmentedrigidarm;

    /** @brief Coordinate parametrization of all variables */
    CoordType coord_type = CoordType::thetax;

    /** @brief All sensors which are to be used */
    std::vector<SensorType> sensors = {SensorType::qualisys};

    /** @brief Controller */
    ControllerType controller_type = ControllerType::osc;

    /** @brief Filtering type between multiple sensor inputs */
    FilterType filter_type = FilterType::none;

    /** @brief Degrees of freedom of arm. is set when finalize() is called */
    int q_size;

    /** @brief Extra objects to be tracked by sensors */
    int objects = 0;

    /** @brief Sensor refresh rate in hz 
     * @details Some sensors may be restricted to lower polling rates automatically due to hardware restrictions
    */
    double sensor_refresh_rate = 100.;

    /** @brief Speed at which model self-updates, given in hz */
    double model_update_rate = 100.;

    /** @brief Controller refresh rate in hz */
    double controller_update_rate = 50;

    /** @brief Serial address of the BendLabs sensors (if using) */
    std::string bendlabs_address = "/dev/ttyACM0";

    /** @brief Size of the pseudo pressure vector
     * @details The pseudo-pressure vector is of same size as q_size, to avoid underaction. You can transform back to "real" pressure with Model::pseudo2real */
    int p_pseudo_size;

    /** @brief Size of the real pressure vector */
    int p_size;

    /** @brief Determines if there is an additional prismatic joint at the base of the robot */
    bool prismatic = false;

    /** @brief Maximal pressure for the valve controller */
    int p_max = 650;

    /** @brief Coefficients for an angular offset polynomial. currently not used. */
    std::vector<double> angOffsetCoeffs = {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0};

    /** @brief A matrix configurations. Characterized from script. */
    std::vector<double> chamberConfigs = {-1, 0.5, -0.5, 0, sqrt(3)/2, -sqrt(3)/2, -1, 0.5, -0.5, 0, sqrt(3)/2, -sqrt(3)/2};

    void finalize(){
        assert(!is_finalized()); 

        // run sanity checks to make sure that at least the size of the arrays make sense
        assert(num_segments * 2 == masses.size());
        assert(num_segments * 2 == lengths.size());
        assert(num_segments + 1 == diameters.size());
        assert(num_segments == shear_modulus.size());
        assert(num_segments == drag_coef.size());
        assert(chamberConfigs.size() == num_segments*6);
        assert(angOffsetCoeffs.size() == num_segments*12);
        p_size = 3*num_segments+2*prismatic+1;
        q_size = 2*num_segments*sections_per_segment+prismatic;
        p_pseudo_size = q_size;
        finalized = true;
    }

    /** @brief populate the parameters by reading from a YAML file
     * @details the YAML file should be placed inside of the config folder to be read
     */
    void load_yaml(const std::string filename){
    assert(!is_finalized());
    std::string loc = fmt::format("{}/config/{}",SOFTTRUNK_PROJECT_DIR,filename);
    YAML::Node params = YAML::LoadFile(loc);
    this->robot_name = params["robot name"].as<std::string>();
    this->num_segments = params["num segments"].as<int>();
    this->sections_per_segment = params["sections per segment"].as<int>();
    this->masses = params["masses"].as<std::vector<double>>();
    this->lengths = params["lengths"].as<std::vector<double>>();
    this->diameters = params["diameters"].as<std::vector<double>>();
    this->armAngle = params["armAngle"].as<double>();
    this->shear_modulus = params["shear modulus"].as<std::vector<double>>();
    this->drag_coef = params["drag coef"].as<std::vector<double>>();  
    this->valvemap = params["valvemap"].as<std::vector<int>>();
    this->sensor_refresh_rate = params["sensor refresh rate"].as<double>();
    this->bendlabs_address = params["bendlabs address"].as<std::string>();
    this->model_update_rate = params["model update rate"].as<double>();
    this->chamberConfigs = params["chamberConfigs"].as<std::vector<double>>();
    this->p_max = params["p_max"].as<int>();
    this->prismatic = params["prismatic"].as<bool>();

    std::vector<std::string> sensor_vec = params["sensors"].as<std::vector<std::string>>();
    this->sensors.clear();
    for (int i = 0; i < sensor_vec.size(); i++){
        if (sensor_vec[i]=="qualisys"){
            sensors.push_back(SensorType::qualisys);
            continue;
        }
        if (sensor_vec[i]=="bendlabs"){
            sensors.push_back(SensorType::bendlabs);
            continue;
        }
        if (sensor_vec[i]=="simulator"){
            sensors.push_back(SensorType::simulator);
            continue;
        }
        fmt::print("Error reading sensors from YAML!\n");
        assert(false);
    }
    
    std::string modeltype = params["model type"].as<std::string>();
    if (modeltype == "augmented"){
        model_type = ModelType::augmentedrigidarm;
    } else if (modeltype == "lagrange"){
        model_type = ModelType::lagrange;
    } else {
        fmt::print("Error reading model type from YAML!\n");
        assert(false);
    }
    std::string coordtype = params["coord_type"].as<std::string>();
    if (coordtype == "thetax"){
        coord_type = CoordType::thetax;
    } else if (coordtype == "phitheta"){
        coord_type = CoordType::phitheta;
    } else {
        fmt::print("Error reading coordinate type from YAML!\n");
        assert(false);
    }

    if (sensor_refresh_rate < model_update_rate){
        model_update_rate = sensor_refresh_rate;
    }
    }

    /** @brief Spit out all parameters into a YAML file
     * @details The created YAML is placed inside the config folder */
    void write_yaml(const std::string filename){
    assert(is_finalized());
    std::string loc = fmt::format("{}/config/{}",SOFTTRUNK_PROJECT_DIR,filename);
    YAML::Node params;
    params["robot name"] = this->robot_name;
    params["num segments"] = this->num_segments;
    params["sections per segment"] = this-> sections_per_segment;
    params["masses"] = this->masses;
    params["masses"].SetStyle(YAML::EmitterStyle::Flow); //flow style makes it appear as a vector instead of indented
    params["lengths"] = this->lengths;
    params["lengths"].SetStyle(YAML::EmitterStyle::Flow);
    params["diameters"] = this->diameters;
    params["diameters"].SetStyle(YAML::EmitterStyle::Flow);
    params["armAngle"] = this->armAngle;
    params["shear modulus"] = this->shear_modulus;
    params["shear modulus"].SetStyle(YAML::EmitterStyle::Flow);
    params["drag coef"] = this->drag_coef;
    params["drag coef"].SetStyle(YAML::EmitterStyle::Flow);
    params["valvemap"] = this->valvemap;
    params["valvemap"].SetStyle(YAML::EmitterStyle::Flow);  
    params["sensor refresh rate"] = this->sensor_refresh_rate;
    params["bendlabs address"] = this->bendlabs_address;
    params["model update rate"] = this->model_update_rate;
    params["chamberConfigs"] = this->chamberConfigs;
    params["chamberConfigs"].SetStyle(YAML::EmitterStyle::Flow);
    params["p_max"] = this->p_max;
    params["prismatic"] = this->prismatic;
    std::vector<std::string> sensor_vec;
    for (int i = 0; i < this->sensors.size(); i++){
        if (sensors[i]==SensorType::qualisys){
            sensor_vec.push_back("qualisys");
            continue;
        }
        if (sensors[i]==SensorType::bendlabs){
            sensor_vec.push_back("bendlabs");
            continue;
        }
        if (sensors[i]==SensorType::simulator){
            sensor_vec.push_back("simulator");
            continue;
        }
        assert(false);
    }
    params["sensors"] = sensor_vec;
    params["sensors"].SetStyle(YAML::EmitterStyle::Flow); 

    std::string model;
    if(this->model_type == ModelType::augmentedrigidarm){
        model = "augmented";
    } else if (this->model_type == ModelType::lagrange){
        model = "lagrange";
    } else {
        assert(false);
    }
    params["model type"] = model;

    std::ofstream out(loc);
    out << "---\n";
    out << "#This file is autogenerated and therefore does not contain documentation of the parameters\n";
    out << "#See the README in the config folder for descriptions on each parameter\n";
    out << params;
    out << "\n...";
    }
    
    bool is_finalized() const {
        return finalized;
    }
private:
    bool finalized = false;
};

/** @brief convert from phi-theta parametrization to longitudinal, for a single PCC section. */
inline void phiTheta2longitudinal(double phi, double theta, double& Lx, double& Ly){
    Lx = -cos(phi) * theta;
    Ly = -sin(phi) * theta;
}

/** @brief convert from longitudinal to phi-theta parametrization, for a single PCC section. */
inline void longitudinal2phiTheta(double Lx, double Ly, double& phi, double& theta){
    phi = atan2(-Ly, -Lx);
    if (Lx == 0 && Ly == 0)
        phi = 0;
    theta = sqrt(pow(Lx, 2) + pow(Ly, 2));
}