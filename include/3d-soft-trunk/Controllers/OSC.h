#pragma once

#include "3d-soft-trunk/ControllerPCC.h"

class PotentialField{
public:
    PotentialField();
    PotentialField(Vector3d &pos, double strength);

    /** @brief get acceleration caused by potential field on an object at position pos 
    * @param pos cartesian position of object being influenced by field
    */
    Vector3d get_ddx(Vector3d &pos);

    /** @brief get position of potential field's center */
    Vector3d get_pos();
    /** @brief get potential field gain */
    double get_strength();
    /** @brief get radius at which potential field stops influencing objects */
    double get_cutoff();

    /** @brief set position of potential field center */
    void set_pos(Vector3d &pos);
    /** @brief set potential field gain */
    void set_strength(double s);
    /** @brief set radius at which potential field stops influencing objects */
    void set_cutoff(double c);
    /** @brief set radius from center at which potential field begins */
    void set_radius(double r);

private:
    Vector3d pos;
    double strength;
    double cutoff_distance;
    double radius;
};

class OSC: public ControllerPCC
{
public:

    OSC(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects = 0);

    /* @brief vector containing all potential fields */
    std::vector<PotentialField> potfields;

    /** @brief get kp gain */    
    double get_kp();
    /** @brief get kd gain */
    double get_kd();

    /** @brief set kp gain */
    void set_kp(double kp);
    /** @brief set kd gain */
    void set_kd(double kd);

    bool freeze = false;


private:
    
    void control_loop();

    /** @brief checks if the given jacobian is in a singularity
     *  @return order of singularity
     *  @details you can also use this as a bool
     */ 
    int singularity(const MatrixXd &J);


    /** @brief gains for OSC*/
    double kp;
    double kd;

    /** @brief x coordinates of middle segment tip */
    Vector3d x_mid;

    /** @brief operational dynamics */
    MatrixXd B_op;
    /** @brief operational space gravity*/
    VectorXd g_op;
    /** @brief extended jacobian inverse*/
    MatrixXd J_inv;
    /** @brief jacobian */
    MatrixXd J;
    /** @brief Jacobian of both middle segment and tip segment */
    MatrixXd J_mid;

    /** @brief reference torques */
    VectorXd tau_ref;
    /** @brief nullspace torques */
    VectorXd tau_null;
    /** @brief reference acceleration in cartesian coordinates */
    Vector3d ddx_des;

    /** @brief ddx for null space control */
    VectorXd ddx_null;
    MatrixXd B_op_null;
    VectorXd f_null;

    MatrixXd computePinv(MatrixXd j,double e,double lambda);
    
};