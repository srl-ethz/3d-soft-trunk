#pragma once

#include "SoftTrunk_common.h"
#include "AugmentedRigidArm.h"

/**
 * @brief dynamic analytical model of the Soft Trunk. Refer to Toshimitsu&Wong2021 for background theory.
 * 
 */
class SoftTrunkModel{
public:
    SoftTrunkModel(){
        ara = std::make_unique<AugmentedRigidArm>();
    };
    /**
     * @brief get the matrices & vectors in the dynamic model for the current state.
     * B \ddot q + C \dot q + g + K q + D \dot q = A p
     */
    void getModel(MatrixXd& B, MatrixXd& C, VectorXd& g, MatrixXd& K, MatrixXd& D, MatrixXd& A);

    void updateState(const VectorXd& q, const VectorXd& dq);
private:
    std::unique_ptr<AugmentedRigidArm> ara;
    /**
     * @brief calculate various properties of a cross section of the arm.
     * @param radius radius of the chamber. This is the input from which the other values will be calculated.
     * @param chamberCentroidDist distance between center & centroid of chamber (i.e. moment arm of pressure force)
     * @param siliconeArea area of silicone cross section
     * @param chamberArea area of a single chamber
     * @param secondMomentOfArea
     */
    void calculateCrossSectionProperties(double radius, double& chamberCentroidDist, double& siliconeArea, double& chamberArea, double& secondMomentOfArea);
};