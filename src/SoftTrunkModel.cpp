#include <3d-soft-trunk/SoftTrunkModel.h>

const double pi = 3.14159265;

SoftTrunkModel::SoftTrunkModel()
{
    ara = std::make_unique<AugmentedRigidArm>();
    double centroidDist;
    double siliconeArea;
    double chamberArea;
    double secondMoment;
    calculateCrossSectionProperties(17.5, centroidDist, siliconeArea, chamberArea, secondMoment);
    fmt::print("centroid:{}, area of silicone:{}, area of single chamber:{}, secondMoment:{}", centroidDist, siliconeArea, chamberArea, secondMoment);

    K = MatrixXd::Zero(2 * st_params::sections_per_segment * st_params::num_segments, 2 * st_params::sections_per_segment * st_params::num_segments);
    D = MatrixXd::Zero(2 * st_params::sections_per_segment * st_params::num_segments, 2 * st_params::sections_per_segment * st_params::num_segments);
    A = MatrixXd::Zero(2 * st_params::sections_per_segment * st_params::num_segments, 3 * st_params::num_segments);

    MatrixXd chamberMatrix = MatrixXd::Zero(2, 3); // describes the direction of each chamber
    chamberMatrix << 1, -0.5, -0.5, 0, sqrt(3) / 2, -sqrt(3) / 2;
    for (int section_id = 0; section_id < st_params::sections_per_segment * st_params::num_segments; section_id++)
    {
        int segment_id = section_id / st_params::sections_per_segment;
        int section_id_in_segment = section_id % st_params::sections_per_segment;
        double r_top = st_params::diameters[segment_id];        // radius at base of segment
        double r_bottom = st_params::diameters[segment_id + 1]; // radius at tip of segment
        // radius at this section
        double radius = (r_top * (st_params::sections_per_segment - section_id_in_segment) + r_bottom * section_id_in_segment) / (double)st_params::sections_per_segment / 2.;
        double chamberCentroidDist;
        double siliconeArea;
        double chamberArea;
        double secondMomentOfArea;
        double l = st_params::lengths[2 * segment_id] / st_params::sections_per_segment; // length of section
        fmt::print("section ID:{} = segment{} * N + {}\n", section_id, segment_id, section_id_in_segment);
        fmt::print("r: {}, l:{}\n", radius, l);
        calculateCrossSectionProperties(radius, chamberCentroidDist, siliconeArea, chamberArea, secondMomentOfArea);

        K.block(2 * section_id, 2 * section_id, 2, 2) = MatrixXd::Identity(2, 2) * 4 * shear_modulus / l;
        A.block(2 * section_id, 3 * segment_id, 2, 3) = chamberArea * chamberCentroidDist * chamberMatrix; 
    }

}

void SoftTrunkModel::updateState(const VectorXd &q, const VectorXd &dq)
{
    ara->update(q, dq);
    B = ara->Jm.transpose() * ara->B_xi * ara->Jm;
    C = ara->Jm.transpose() * ara->B_xi * ara->dJm;
    g = ara->Jm.transpose() * ara->G_xi;
    J = ara->Jxi * ara->Jm;
    /** @todo do for K & D, then it should be done!! */
}
void SoftTrunkModel::calculateCrossSectionProperties(double radius, double &chamberCentroidDist, double &siliconeArea, double &chamberArea, double &secondMomentOfArea)
{
    double r1 = radius - 2;                                                           /** the radius of the cavity */
    double r2 = radius;                                                               /** radius of the outer shell */
    double r3 = 1.7;                                                                  /** radius of the tubes */
    double l1 = 2 * radius;                                                           /** length of the triangle. FIX to consider the fillets! */
    double b = 11.37;                                                                 /** distance from the center of tube to the centroid */
    double k1 = 4 * (pow(r2, 3) - pow(r1, 3)) / (3 * pi * (pow(r2, 2) - pow(r1, 2))); /** distance from the centroid of the chamber to the baseline of the section */
    double k2 = k1 + l1 / 2 / sqrt(3);                                                /** distance from the centroid of the chamber to the centroid of the section */

    /** Moment of Inertia of the middle triangle (same in x & y directions) */
    double I_triangle = sqrt(3) * pow(l1, 4) / 96;

    /** @todo the tubes are skipped for now... */

    // moment of inertia of the rectangle
    double I_rect_y = 4 * pow(r1, 3) / 3;
    double I_rect_x = 4 / 3 * r1 + 4 * r1 * pow(1 + k2 - k1, 2);

    // moment of inertia of the chamber
    double I_chamber_y = pi * (pow(r2, 4) - pow(r1, 4)) / 8;
    double I_chamber_x = pi * (pow(r2, 4) - pow(r1, 4)) / 8 - pow(k1, 2) * (pow(r2, 2) - pow(r1, 2)) * pi / 2 + pow(k2, 2) * (pow(r2, 2) - pow(r1, 2)) * pi / 2;

    chamberArea = pow(r1, 2) * pi / 2 - 2 * r1 * 2;
    chamberCentroidDist = l1 / 2 / sqrt(3) + (4 * pow(r1, 2) - 24) / (3 * r1 * pi - 24);
    siliconeArea = 3 * (pow(r2, 2) * pi / 2 - chamberArea) + sqrt(3) / 4 * pow(l1, 2); // 3 * (area of silicone in chamber) + (triangle) - (tubes)
    // uses RotationMatrix[theta].{{Iy, 0},{0, Ix}}.RotationMatrix[theta]^T to calculate for rotated values of chamber & rectangles (at 2pi/3 and 4pi/3 rotations)
    secondMomentOfArea = I_triangle + I_rect_y + (I_rect_y / 2 + I_rect_x * 1.5) + I_chamber_y + (I_chamber_y / 2 + I_chamber_x * 1.5); /** @todo verify this */
}