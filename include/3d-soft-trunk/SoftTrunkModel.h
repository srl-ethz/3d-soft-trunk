#pragma once

#include "SoftTrunk_common.h"
#include "AugmentedRigidArm.h"
#include <fstream>

/**
 * @brief dynamic analytical model of the Soft Trunk. Refer to IROS2021 paper (toshimitsu et al., 2021) for background theory.
 * The matrices returned describe the following model:
 * \f$
 * B \ddot q + c + g + K q + D \dot q = A p
 * \f$
 * where q is the pose (in longitudinal parameters) with dimension of 2 * num_segments * sections_per_segment,
 * and p is input pressures with dimension of 2 * num_segments. p[0] is chamber in +x direction, p[1] is chamber in -x,+y direction and p[2] is chamber in -x,-y direction
 */
class SoftTrunkModel{
public:
    SoftTrunkModel(const SoftTrunkParameters& st_params);

    /**
     * @brief update the model's state, and calculate the parameters of the model. Currently has problems when both Lx and Ly are 0.
     */
    void updateState(const srl::State &state);
    SoftTrunkParameters getSoftTrunkParameters(){
        return st_params;
    }

    /** @brief inertia matrix */
    MatrixXd B;

    /** @brief coreolis & centrifugal force */
    VectorXd c;

    /** @brief gravity */
    VectorXd g;

    /** @brief elastic coefs */
    MatrixXd K;

    /** @brief dissipative coefs */
    MatrixXd D;

    /** @brief map from pressure to generalized force */
    MatrixXd A;

    /** @brief map from pseudopressure to generalized force.
     * The concept of pseudopressure creates a virtual chamber aligned with X and Y axes that can also output negative pressure values.
     * Calculating using pseudopressure may make it easier for some controllers.
    */
    MatrixXd A_pseudo;
    
    /** @brief the Jacobian gives the relation between the pose \f$q\f$ and tip position \f$x\f$ in global coordinates. */
    std::vector<Eigen::MatrixXd> J;

    /**
     * @brief get relative pose from base to tip of segment (at the tip of curved sections, ignores connector part)
     * @param segment_id 0 for first segment, and so on
     */
    Eigen::Transform<double, 3, Eigen::Affine> get_H(int segment_id);

    Eigen::Transform<double, 3, Eigen::Affine> get_H_base();


    /** @brief set new angles for the chamber configuration*/
    void newChamberConfig(Vector3d &angles);
    
    /**
     * @brief convert pseudopressures to real pressures
     * WARNING: Does not perform any unit conversion, so give it pseudopressures in mbar (or convert to mbar after 2 -> 3 conversion) 
     * @param pressure_pseudo 2d input pressure
     * @return VectorXd of 3d output pressure
     */
    VectorXd pseudo2real(VectorXd pressure_pseudo);


    std::unique_ptr<AugmentedRigidArm> ara;
private:
    

private:
    const SoftTrunkParameters st_params;
    /**
     * @brief calculate various properties of a cross section of the arm. All units of input / output are in meters.
     * @param radius radius of the chamber. This is the input from which the other values will be calculated.
     * @param chamberCentroidDist distance between center & centroid of chamber (i.e. moment arm of pressure force)
     * @param siliconeArea area of silicone cross section
     * @param chamberArea area of a single chamber (its empty space)
     * @param secondMomentOfArea
     */
    void calculateCrossSectionProperties(double radius, double& chamberCentroidDist, double& siliconeArea, double& chamberArea, double& secondMomentOfArea);

    /**
     * @brief generates URDF model of robot as configured in SoftTrunk_common.h. It is then read by the AugmentedRigidArm class.
     */
    void generateRobotURDF();

    /** @brief mapping between 2D pseudopressures and 3 chambers */
    MatrixXd chamberMatrix = MatrixXd::Zero(2, 3); // describes the direction of each chamber
    MatrixXd rc = MatrixXd::Zero(30,5);
    VectorXd rca = VectorXd::Zero(31);
};
