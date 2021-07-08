#include <3d-soft-trunk/SoftTrunkModel.h>

const double pi = 3.14159265;

SoftTrunkModel::SoftTrunkModel(const SoftTrunkParameters& st_params): st_params(st_params)
{
    assert(st_params.is_finalized());
    generateRobotURDF();
    ara = std::make_unique<AugmentedRigidArm>(st_params);

    K = MatrixXd::Zero(2 * st_params.sections_per_segment * st_params.num_segments, 2 * st_params.sections_per_segment * st_params.num_segments);
    D = MatrixXd::Zero(2 * st_params.sections_per_segment * st_params.num_segments, 2 * st_params.sections_per_segment * st_params.num_segments);
    A = MatrixXd::Zero(2 * st_params.sections_per_segment * st_params.num_segments, 3 * st_params.num_segments);
    A_pseudo = MatrixXd::Zero(2 * st_params.sections_per_segment * st_params.num_segments, 2*st_params.num_segments);
    J.resize(st_params.num_segments);

    chamberMatrix << 1, -0.5, -0.5, 0, sqrt(3) / 2, -sqrt(3) / 2;
    for (int section_id = 0; section_id < st_params.sections_per_segment * st_params.num_segments; section_id++)
    {
        int segment_id = section_id / st_params.sections_per_segment;
        int section_id_in_segment = section_id % st_params.sections_per_segment;
        double r_top = st_params.diameters[segment_id]/2;        // radius at base of segment
        double r_bottom = st_params.diameters[segment_id + 1]/2; // radius at tip of segment
        // radius at the middle of this PCC section
        double radius = (r_top * (st_params.sections_per_segment - (0.5+section_id_in_segment)) + r_bottom * (0.5+section_id_in_segment)) / (double)st_params.sections_per_segment;
        double chamberCentroidDist;
        double siliconeArea;
        double chamberArea;
        double secondMomentOfArea;
        double l = st_params.lengths[2 * segment_id] / st_params.sections_per_segment; // length of section
        calculateCrossSectionProperties(radius, chamberCentroidDist, siliconeArea, chamberArea, secondMomentOfArea);

        K.block(2 * section_id, 2 * section_id, 2, 2) = MatrixXd::Identity(2, 2) * 4 * st_params.shear_modulus[segment_id] * secondMomentOfArea / l;
        A.block(2 * section_id, 3 * segment_id, 2, 3) = chamberArea * chamberCentroidDist * chamberMatrix; 
        D.block(2 * section_id, 2 * section_id, 2, 2) = MatrixXd::Identity(2, 2) * secondMomentOfArea * st_params.drag_coef[segment_id] / l; /** this is "heuristic" */
        A_pseudo.block(2 * section_id, 2*segment_id, 2, 2) = chamberArea * chamberCentroidDist * MatrixXd::Identity(2,2);
    }

    rc << 5.00832085631819e-07,-1.97875480711816e-05,0.000227114987019528,0.000325012674886395,0.118202602792403,
-1.02364678438023e-07,4.18561442772814e-06,-5.29397218804607e-05,0.000692818261038357,0.130976648806749,
-2.1120533482029e-09,-7.1424151350523e-07,9.37142192984094e-06,0.000522283345830439,0.136759835916474,
5.06925384016871e-08,-8.15338467105881e-07,-1.80845387221283e-05,0.000425254979592181,0.143084560908916,
-6.16120631060846e-08,1.61114437105488e-06,-3.79982274624382e-06,-1.03653985710035e-05,0.145226052368751,
4.34757167431998e-08,-1.33801971628973e-06,1.1027648067906e-06,0.000168521595165237,0.146055344280887,
-1.44884844855597e-08,7.43017925151421e-07,-9.57751734414206e-06,-8.1896394790947e-05,0.146828556441626,
-6.50589442939748e-08,4.95024677759583e-08,4.6482237089047e-06,-9.12275165915028e-05,0.145453174717658,
2.5055517501946e-07,-3.06465233242894e-06,-4.94737163616164e-05,-0.000404664031554946,0.143777810613513,
-4.02455535622501e-07,8.92858871183587e-06,5.57839416487377e-05,-0.00118787439063345,0.131736953457413,
3.76946521092258e-07,-1.03356162599612e-05,3.05277971598883e-05,0.00122431719542654,0.132557787564956,
-2.98618150196411e-07,7.70755721632157e-06,-1.66458626734425e-05,9.85364332272351e-05,0.141598748526953,
2.35043060784741e-07,-6.58629823974663e-06,3.48073595607764e-06,0.000964441741861395,0.147478542726795,
-1.60236622460706e-07,4.66442960314962e-06,-3.10168060708386e-05,-0.000170631594467641,0.153051521166843,
1.18103508178289e-07,-3.00556339196952e-06,-1.24015758015569e-06,-7.46416660012171e-06,0.151275240004949,
-1.07033047997242e-07,2.64765786616459e-06,-7.66456176835411e-06,-0.000518797757564834,0.148279769795025,
1.10130494719274e-07,-2.47565736463673e-06,-4.57715276592892e-06,-0.000298458058013475,0.143316169098548,
-1.20302792601174e-07,2.79592231592584e-06,1.17160310971051e-06,-0.00071665916934988,0.137105179650008,
1.49522868653525e-07,-2.9625713565837e-06,-1.81974717009822e-06,-0.000312103947157686,0.131021141116869,
-1.7369023214596e-07,4.19458995629837e-06,2.029498669478e-05,-0.000603473640382079,0.125015126141193,
1.27473103167809e-07,-4.11938248908825e-06,2.16449607312018e-05,0.000493692776445434,0.124326051008308,
-6.35117914287514e-08,1.98233004921087e-06,-1.67151305645972e-05,0.000115800403189955,0.12888838648244,
3.01745432130804e-08,-1.05776770051204e-06,-1.19236405453268e-07,0.00013202183293837,0.129975097781363,
4.77874025801513e-09,3.86587101287405e-07,-1.21669281615353e-05,-0.000118419209274769,0.130344027287249,
-2.49125194806813e-08,6.15329468304393e-07,5.81747426263755e-06,-0.000210779088155754,0.127945093072655,
1.49178860005287e-08,-5.77149797504231e-07,6.5027993535006e-06,2.20356973710777e-05,0.126799421898712,
8.6310660407759e-09,1.3691967905442e-07,-1.39933127267343e-06,3.19794275114465e-05,0.127311211376895,
-6.69531977686204e-08,5.5006004020621e-07,1.09319546880551e-05,0.000116472075001289,0.127905137225095,
1.55766740541902e-07,-2.65476635965172e-06,-2.68475237459917e-05,0.000155483115963586,0.130434012999194,
-3.41438729224297e-07,4.80126828762068e-06,1.16821858610512e-05,-0.000559851650379048,0.127094958397189;
    rc << 0,11.9666666666667,23.9333333333333,35.9,47.8666666666667,59.8333333333333,71.8,83.7666666666667,95.7333333333333,107.7,119.666666666667,131.633333333333,143.6,155.566666666667,167.533333333333,179.5,191.466666666667,203.433333333333,215.4,227.366666666667,239.333333333333,251.3,263.266666666667,275.233333333333,287.2,299.166666666667,311.133333333333,323.1,335.066666666667,347.033333333333,359;
}

void SoftTrunkModel::updateState(const srl::State &state)
{
    ara->update(state);
    B = ara->B;
    c = ara->c;
    g = ara->g;
    J = ara->J;
}

Eigen::Transform<double, 3, Eigen::Affine> SoftTrunkModel::get_H(int segment_id){
    return ara->get_H(segment_id);
}
Eigen::Transform<double, 3, Eigen::Affine> SoftTrunkModel::get_H_base(){
    return ara->get_H_base();
}

void SoftTrunkModel::newChamberConfig(Vector3d &angles){
    double toRad = 3.14156/180;
    chamberMatrix << sin(angles(0)*toRad), sin(angles(1)*toRad), sin(angles(2)*toRad), cos(angles(0)*toRad), cos(angles(1)*toRad), cos(angles(2)*toRad);
    fmt::print("Chamber Matrix:\n{}\n", chamberMatrix);
}

VectorXd SoftTrunkModel::pseudo2real(VectorXd pressure_pseudo){
    assert(pressure_pseudo.size() == 2 * st_params.num_segments);
    VectorXd output = VectorXd::Zero(3*st_params.num_segments);
    MatrixXd chamberMatrix_inv = chamberMatrix.transpose()*(chamberMatrix*chamberMatrix.transpose()).inverse(); //old variant
    for (int i = 0; i < st_params.num_segments; i++){
        //constrain the pressure to be 500 at most (this may fuck with your arm if you want more than 600mbar)
        if (pressure_pseudo.segment(2*i,2).norm() > 500) pressure_pseudo.segment(2*i,2) *= 500/pressure_pseudo.segment(2*i,2).norm();

        double angle = atan2(pressure_pseudo(2*i), pressure_pseudo(2*i+1))*180/3.14156;
        if (angle < -30) angle += 360; //-30 because the first region spans -30,90 and this makes that easier
        
        
        //shift coordinates to start at the same spot as the characterization program
        angle = angle - 90; 
        if (angle < -6) angle += 360;       
        double deg2rad = 0.01745329;
        double r = sqrt(pow(pressure_pseudo(2*i),2) + pow(pressure_pseudo(2*i+1),2));
        
        if (-6 < angle && angle <= 4) angle += 2;
        if (4 < angle && angle <= 126) angle += 0.000023019205*pow(angle,3) - 0.002952847129*pow(angle,2) + 0.072823205506*angle + 7.271932801016;


        else if (126 < angle && angle < 240) angle += -0.000005370720*pow(angle-126,3) + 0.000312050650*pow(angle-126,2) + 0.010738591328*(angle-126) + 13.832115039876;


        else if (240 < angle && angle <= 360) angle += 0.000041666881*pow(angle-240,3) - 0.009056094721*pow(angle-240,2) + 0.437521763515*(angle-240) + 12.346903014262;

        //excel is a motherfucker for making us do the -232
        


        pressure_pseudo(2*i) = r*cos(angle*deg2rad);
        pressure_pseudo(2*i+1) = -r*sin(angle*deg2rad);

        output.segment(3*i, 3) = chamberMatrix_inv * pressure_pseudo.segment(2*i, 2); //invert back onto real chambers

        double min_p = output.segment(3*i, 3).minCoeff();
        if (min_p < 0)
            output.segment(3*i, 3) -= min_p * Vector3d::Ones(); //remove any negative pressures, as they are not physically realisable

        if (angle < 0) angle += 360;
        //these values are obtained from manual curve fitting on the data from radial pressure distribution (see Characterize)
        
        /*if (0 < angle && angle <= 118) output.segment(3*i,3) *= 0.14/(-0.000000006902*pow(angle,3) - 0.000005643435*pow(angle,2) + 0.000830297530*angle + 0.119540588926);
        else if (118 < angle && angle <= 234) output.segment(3*i,3) *= 0.14/(0.000000060972*pow(angle-118,3) - 0.000017514232*pow(angle-118,2) + 0.001139288022*(angle-118) + 0.130108516811);
        else if (234 < angle && angle <=360) output.segment(3*i,3) *= 0.14/(0.000000000000472113*pow(angle-234,6) - 0.000000000178955503*pow(angle-234,5) + 0.000000024484785244*pow(angle-234,4) - 0.000001441192959112*pow(angle-234,3) + 0.000031911305931942*pow(angle-234,2) - 0.000064605943293827*(angle-234) + 0.126783514275473000);
        */
        for (int i = 0; i < 30; i++){
            if (angle > rca(i)){
                output.segment(3*i,3) *= 0.14/(rc(0,i)*pow(angle-rca(i),4) + rc(1,i)*pow(angle-rca(i),3) + rc(2,i)*pow(angle-rca(i),2) + rc(3,i)*pow(angle-rca(i),1) + rc(4,i)*pow(angle-rca(i),0));
                break;
            }
        }
    }
    return output;
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
    std::string xacro_filename = fmt::format("{}/urdf/{}.urdf.xacro", SOFTTRUNK_PROJECT_DIR, st_params.robot_name);
    std::string urdf_filename = fmt::format("{}/urdf/{}.urdf", SOFTTRUNK_PROJECT_DIR, st_params.robot_name);

    // sanity check of the parameters, just in case
    assert(2 * st_params.num_segments == st_params.lengths.size());
    assert(st_params.num_segments + 1 == st_params.diameters.size());

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
               << fmt::format("<robot xmlns:xacro='http://www.ros.org/wiki/xacro' name='{}'>\n", st_params.robot_name)
               << "<xacro:include filename='macro_definitions.urdf.xacro' />\n"
               << "<xacro:empty_link name='base_link'/>\n"
               << fmt::format("<xacro:rigid_rotation rotX='0' rotY='{}' rotZ='0' parent='base_link' child='softTrunk_base'/>", st_params.armAngle*pi/180)
               << "<xacro:empty_link name='softTrunk_base'/>\n";

    std::string parent = "softTrunk_base";
    std::string child;
    for (int i = 0; i < st_params.num_segments; i++)
    {
        // create sections that gradually taper
        double segmentLength = st_params.lengths[2*i];
        double connectorLength = st_params.lengths[2*i+1];
        double baseRadius = st_params.diameters[i]/2.;
        double tipRadius = st_params.diameters[i+1]/2.;
        double sectionLengthInSegment = segmentLength / st_params.sections_per_segment; // length of a PCC section within a segment
        double segmentMass = st_params.masses[2*i];
        double connectorMass = st_params.masses[2*i+1];
        double dragon_skin_10_density = 1e6*1.07;

        double sectionLength;
        double sectionVolume;
        double sectionMass;

        double segmentVolume = 0;
        // first calculate the segment's model volume (excluding connector piece)
        for (int j = 0; j < st_params.sections_per_segment; j++)
        {
            double sectionRadius = (tipRadius * j + baseRadius * (st_params.sections_per_segment-j))/st_params.sections_per_segment;
            calculateCrossSectionProperties(sectionRadius, tmp1, siliconeArea, singleChamberArea, tmp2);
            segmentVolume += siliconeArea * sectionLengthInSegment;
        }
        fmt::print("estimated volume of segment {} is {} m^3, i.e. {} g. Actual value is {}g.\n", i, segmentVolume, segmentVolume*dragon_skin_10_density, segmentMass*1e3); // Dragon Skin 10 is 1.07g/cc

        for (int j = 0; j < st_params.sections_per_segment + 1; j++)
        {
            // there is an "extra" PCC section at the end of each segment, to represent the straight connector piece that will always be kept straight.
            double sectionRadius = (tipRadius * j + baseRadius * (st_params.sections_per_segment-j))/st_params.sections_per_segment;
            calculateCrossSectionProperties(sectionRadius, tmp1, siliconeArea, singleChamberArea, tmp2);
            child = fmt::format("seg{}_sec{}-{}_connect", i, j, j+1);
            if (j != st_params.sections_per_segment){
                sectionLength = sectionLengthInSegment;
                sectionVolume = siliconeArea * sectionLength;
                // distribute measured total mass equally by modelled volume
                sectionMass = segmentMass * sectionVolume / segmentVolume;
            }
            else{
                sectionLength = connectorLength; // this is the connection piece which is for implementation represented as another PCC section.
                sectionVolume = (siliconeArea + 3 * singleChamberArea) * sectionLength;
                sectionMass = connectorMass;
                fmt::print("estimated volume of connector at tip of segment {} is {} m^3, i.e. {}g. Actual value is {}g.\n", i, sectionVolume, sectionVolume*dragon_skin_10_density, connectorMass*1e3);
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
