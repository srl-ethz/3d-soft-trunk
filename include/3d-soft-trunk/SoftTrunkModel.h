#pragma once

#include "SoftTrunk_common.h"
#include "AugmentedRigidArm.h"

/**
 * @brief dynamic analytical model of the Soft Trunk. Refer to Toshimitsu&Wong2021 for background theory.
 * The matrices returned describe the following model:
 * \f$
 * B \ddot q + C \dot q + g + K q + D \dot q = A p
 * \f$
 * where q is the pose (in longitudinal parameters) with dimension of 2 * num_segments * sections_per_segment,
 * and p is input pressures with dimension of 2 * num_segments.
 */
class SoftTrunkModel{
public:
    SoftTrunkModel(){
        ara = std::make_unique<AugmentedRigidArm>();
        double centroidDist;
        double siliconeArea;
        double chamberArea;
        double secondMoment;
        calculateCrossSectionProperties(17.5, centroidDist, siliconeArea, chamberArea, secondMoment);
        fmt::print("centroid:{}, area of silicone:{}, area of single chamber:{}, secondMoment:{}", centroidDist, siliconeArea, chamberArea, secondMoment);
    };

    /**
     * @brief update the model's state, and calculate the parameters of the model.
     */
    void updateState(const VectorXd& q, const VectorXd& dq);

    /** @brief inertia matrix */
    MatrixXd B;

    /** @brief coreolis & centrifugal force */
    MatrixXd C;

    /** @brief gravity */
    VectorXd g;

    /** @brief elastic coefs */
    MatrixXd K;

    /** @brief dissipative coefs */
    MatrixXd D;

    /** @brief map from pressure to generalized force */
    MatrixXd A;
    
    /** @brief the Jacobian gives the relation between the pose \f$q\f$ and tip position \f$x\f$. */
    MatrixXd J;
    
private:
    std::unique_ptr<AugmentedRigidArm> ara;
    /**
     * @brief calculate various properties of a cross section of the arm.
     * @param radius radius of the chamber. This is the input from which the other values will be calculated.
     * @param chamberCentroidDist distance between center & centroid of chamber (i.e. moment arm of pressure force)
     * @param siliconeArea area of silicone cross section
     * @param chamberArea area of a single chamber (its empty space)
     * @param secondMomentOfArea
     */
    void calculateCrossSectionProperties(double radius, double& chamberCentroidDist, double& siliconeArea, double& chamberArea, double& secondMomentOfArea);

};