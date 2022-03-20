#pragma once

#include "3d-soft-trunk/ControllerPCC.h"

/** @brief Potential field to avoid task space objects */
class PotentialField{
public:
    PotentialField();
    PotentialField(Vector3d &pos, double strength);

    /** @brief get acceleration caused by potential field on an object at position pos 
    * @param pos cartesian position of object being influenced by field
    */
    Vector3d get_ddx(Vector3d &pos);

    Vector3d pos_;
    double strength_;
    double cutoff_distance_;
    double radius_;
};

/** @brief Operational Space Controller
 * @details Used in Fischer 2022 Dynamic */
class OSC: public ControllerPCC
{
public:

    OSC(const SoftTrunkParameters st_params);

    /* @brief vector containing all potential fields */
    std::vector<PotentialField> potfields_;

    bool freeze = false;

    /** @brief proportional gain */
    double kp_;

    /** @brief derivative gain */
    double kd_;

private:
    void control_loop();

    /** @brief operational dynamics */
    MatrixXd B_op;
    /** @brief operational space gravity*/
    VectorXd g_op;
    /** @brief extended jacobian inverse*/
    MatrixXd J_inv;
    /** @brief jacobian */
    MatrixXd J;

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

    /** @brief pseudoinverse of a matrix */
    MatrixXd computePinv(MatrixXd j,double e,double lambda);
    
};