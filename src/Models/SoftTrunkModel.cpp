#include <3d-soft-trunk/Models/SoftTrunkModel.h>

const double pi = 3.14159265;

SoftTrunkModel::SoftTrunkModel(const SoftTrunkParameters& st_params): st_params_(st_params)
{
    assert(st_params_.is_finalized());
    generateRobotURDF();
    ara = std::make_unique<AugmentedRigidArm>(st_params);

    dyn_.coordtype = st_params_.coord_type;
    dyn_.K = MatrixXd::Zero(st_params_.q_size, st_params_.q_size);
    dyn_.D = MatrixXd::Zero(st_params_.q_size, st_params_.q_size);
    dyn_.A = MatrixXd::Zero(st_params_.q_size, st_params_.p_size);
    dyn_.A_pseudo = MatrixXd::Zero(st_params_.q_size, st_params_.p_pseudo_size);
    dyn_.J.resize(st_params_.num_segments);
    chamberMatrix << 1, -0.5, -0.5, 0, sqrt(3) / 2, -sqrt(3) / 2;

    for (int section_id = 0; section_id < st_params_.sections_per_segment * st_params_.num_segments; section_id++)
    {
        int segment_id = section_id / st_params_.sections_per_segment;
        int section_id_in_segment = section_id % st_params_.sections_per_segment;
        double r_top = st_params_.diameters[segment_id]/2;        // radius at base of segment
        double r_bottom = st_params_.diameters[segment_id + 1]/2; // radius at tip of segment
        // radius at the middle of this PCC section
        double radius = (r_top * (st_params_.sections_per_segment - (0.5+section_id_in_segment)) + r_bottom * (0.5+section_id_in_segment)) / (double)st_params_.sections_per_segment;
        double chamberCentroidDist;
        double siliconeArea;
        double chamberArea;
        double secondMomentOfArea;
        double l = st_params_.lengths[2 * segment_id] / st_params_.sections_per_segment; // length of section

        calculateCrossSectionProperties(radius, chamberCentroidDist, siliconeArea, chamberArea, secondMomentOfArea);

        dyn_.K.block(2 * section_id + st_params_.prismatic, 2 * section_id + st_params_.prismatic, 2, 2) = MatrixXd::Identity(2, 2) * 4 * st_params_.shear_modulus[segment_id] * secondMomentOfArea / l;
        dyn_.A.block(2 * section_id + st_params_.prismatic, 3 * segment_id + st_params_.prismatic, 2, 3) = chamberArea * chamberCentroidDist * chamberMatrix; 
        dyn_.D.block(2 * section_id + st_params_.prismatic, 2 * section_id + st_params_.prismatic, 2, 2) = MatrixXd::Identity(2, 2) * secondMomentOfArea * st_params_.drag_coef[segment_id] / l; /** this is "heuristic" */
        dyn_.A_pseudo.block(2 * section_id + st_params_.prismatic, 2*segment_id + st_params_.prismatic, 2, 2) = chamberArea * chamberCentroidDist * MatrixXd::Identity(2,2);
    }
    if (st_params_.prismatic){
        dyn_.A(0,0) = 1;
        dyn_.A_pseudo(0,0) = 1;
    }
    fmt::print("SoftTrunkModel initialized\n");
}

void SoftTrunkModel::set_state(const srl::State &state)
{
    ara->update(state);
    assert(state.coordtype == dyn_.coordtype);
    dyn_.B = ara->B;
    dyn_.c = ara->c;
    dyn_.g = ara->g;
    dyn_.J = ara->J;
    dyn_.S = ara->S;
    xi_ = ara->xi_;
}


Eigen::Transform<double, 3, Eigen::Affine> SoftTrunkModel::get_H(int segment_id){
    return ara->get_H(segment_id);
}
Eigen::Transform<double, 3, Eigen::Affine> SoftTrunkModel::get_H_base(){
    return ara->get_H_base();
}

void SoftTrunkModel::calculateCrossSectionProperties(double radius, double &chamberCentroidDist, double &siliconeArea, double &chamberArea, double &secondMomentOfArea)
{
    radius *= 1000.; // computation is done in mm
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

    // convert to meters
    chamberCentroidDist /= 1000.;
    siliconeArea /= pow(1000., 2);
    chamberArea /= pow(1000., 2);
    secondMomentOfArea /= pow(1000., 4);
}

void SoftTrunkModel::generateRobotURDF(){
    std::string xacro_filename = fmt::format("{}/urdf/{}.urdf.xacro", SOFTTRUNK_PROJECT_DIR, st_params_.robot_name);
    std::string urdf_filename = fmt::format("{}/urdf/{}.urdf", SOFTTRUNK_PROJECT_DIR, st_params_.robot_name);

    // sanity check of the parameters, just in case
    assert(2 * st_params_.num_segments == st_params_.lengths.size());
    assert(st_params_.num_segments + 1 == st_params_.diameters.size());

    fmt::print("generating XACRO file:\t{}\n", xacro_filename);
    std::ofstream xacro_file;

    // calculate total volume, used when calculating mass for each section
    double sectionLength;
    double sectionRadius;
    double tmp1, tmp2;
    double siliconeArea;
    double singleChamberArea;
    
    xacro_file.open(xacro_filename);

    xacro_file << "<?xml version='1.0'?>\n"
               << "<!-- This file has been generated automatically from SoftTrunkModel::generateRobotURDF(), do not edit by hand -->\n"
               << fmt::format("<robot xmlns:xacro='http://www.ros.org/wiki/xacro' name='{}'>\n", st_params_.robot_name)
               << "<xacro:include filename='macro_definitions.urdf.xacro' />\n"
               << "<xacro:empty_link name='base_link'/>\n"
               << fmt::format("<xacro:rigid_rotation rotX='0' rotY='{}' rotZ='0' parent='base_link'", st_params_.armAngle*pi/180);

    std::string parent = "softTrunk_base";
    std::string child;

    if(st_params_.prismatic){
        xacro_file << " child='prismaticSoPrA'/>\n"
        << "<xacro:empty_link name='prismaticSoPrA'/>\n";
        xacro_file << fmt::format("<xacro:prismatic id='prismaticSoPrA' parent='base_link' child='softTrunk_base' length='0.15' mass='0.001' radius='0.03'/>\n");
    } else {
        xacro_file << " child='softTrunk_base'/>\n";
    }

    xacro_file << "<xacro:empty_link name='softTrunk_base'/>\n";
    for (int i = 0; i < st_params_.num_segments; i++)
    {
        // create sections that gradually taper
        double segmentLength = st_params_.lengths[2*i];
        double connectorLength = st_params_.lengths[2*i+1];
        double baseRadius = st_params_.diameters[i]/2.;
        double tipRadius = st_params_.diameters[i+1]/2.;
        double sectionLengthInSegment = segmentLength / st_params_.sections_per_segment; // length of a PCC section within a segment
        double segmentMass = st_params_.masses[2*i];
        double connectorMass = st_params_.masses[2*i+1];
        double dragon_skin_10_density = 1e6*1.07;

        double sectionLength;
        double sectionVolume;
        double sectionMass;

        double segmentVolume = 0;
        // first calculate the segment's model volume (excluding connector piece)
        for (int j = 0; j < st_params_.sections_per_segment; j++)
        {
            double sectionRadius = (tipRadius * j + baseRadius * (st_params_.sections_per_segment-j))/st_params_.sections_per_segment;
            calculateCrossSectionProperties(sectionRadius, tmp1, siliconeArea, singleChamberArea, tmp2);
            segmentVolume += siliconeArea * sectionLengthInSegment;
        }
        //fmt::print("estimated volume of segment {} is {} m^3, i.e. {} g. Actual value is {}g.\n", i, segmentVolume, segmentVolume*dragon_skin_10_density, segmentMass*1e3); // Dragon Skin 10 is 1.07g/cc

        for (int j = 0; j < st_params_.sections_per_segment + 1; j++)
        {
            // there is an "extra" PCC section at the end of each segment, to represent the straight connector piece that will always be kept straight.
            double sectionRadius = (tipRadius * j + baseRadius * (st_params_.sections_per_segment-j))/st_params_.sections_per_segment;
            calculateCrossSectionProperties(sectionRadius, tmp1, siliconeArea, singleChamberArea, tmp2);
            child = fmt::format("seg{}_sec{}-{}_connect", i, j, j+1);
            if (j != st_params_.sections_per_segment){
                sectionLength = sectionLengthInSegment;
                sectionVolume = siliconeArea * sectionLength;
                // distribute measured total mass equally by modelled volume
                sectionMass = segmentMass * sectionVolume / segmentVolume;
            }
            else{
                sectionLength = connectorLength; // this is the connection piece which is for implementation represented as another PCC section.
                sectionVolume = (siliconeArea + 3 * singleChamberArea) * sectionLength;
                sectionMass = connectorMass;
                //fmt::print("estimated volume of connector at tip of segment {} is {} m^3, i.e. {}g. Actual value is {}g.\n", i, sectionVolume, sectionVolume*dragon_skin_10_density, connectorMass*1e3);
            }
            xacro_file << fmt::format("<xacro:PCC id='seg{}_sec{}' parent='{}' child='{}' length='{}' mass='{}' radius='{}'/>\n", i, j, parent, child, sectionLength, sectionMass, sectionRadius);
            xacro_file << fmt::format("<xacro:empty_link name='{}'/>\n", child);
            parent = child;
        }
    }
    xacro_file << "</robot>";

    xacro_file.close();

    fmt::print("generating URDF file (XACRO must be installed):\t{}\n", urdf_filename);
    if (0 != std::system(fmt::format("python3 {}/urdf/xacro2urdf.py {} {}", SOFTTRUNK_PROJECT_DIR, xacro_filename, urdf_filename).c_str()))
        throw "error with xacro -> urdf conversion script, aborting program"; // if python program returns anything other than 0, it is error
    fmt::print("URDF file generated.\n");
}
