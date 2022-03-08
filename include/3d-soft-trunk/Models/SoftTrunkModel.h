#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include "3d-soft-trunk/Models/AugmentedRigidArm.h"
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
    void set_state(const srl::State &state);


    /**
     * @brief get relative pose from base to tip of segment (at the tip of curved sections, ignores connector part)
     * @param segment_id 0 for first segment, and so on
     */
    Eigen::Transform<double, 3, Eigen::Affine> get_H(int segment_id);

    Eigen::Transform<double, 3, Eigen::Affine> get_H_base();

    const SoftTrunkParameters st_params_;
    DynamicParams dyn_;

    /** @brief rigid joint states from augmented rigid arm */
    VectorXd xi_;
    
private:


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

    /** @brief default chamber configuration */
    MatrixXd chamberMatrix;

    std::unique_ptr<AugmentedRigidArm> ara;
};
