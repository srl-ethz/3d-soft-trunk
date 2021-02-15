//
// Created by yasu on 24/10/18.
//
#include "3d-soft-trunk/SoftTrunk_common.h"

#include "fmt/core.h"
#include <fstream>

/**
 * @file create_urdf.cpp
 * @brief generates a URDF model of augmented rigid arm using parameters defined in SoftTrunk_common.h. XACRO (and ROS) must be installed on the system.
 */
int main() {
    std::string xacro_filename = fmt::format("{}/urdf/{}.urdf.xacro", SOFTTRUNK_PROJECT_DIR, st_params::robot_name);
    std::string urdf_filename = fmt::format("{}/urdf/{}.urdf", SOFTTRUNK_PROJECT_DIR, st_params::robot_name);

    assert(2 * st_params::num_segments - 1 == st_params::lengths.size());
    assert(st_params::num_segments + 1 == st_params::diameters.size());

    fmt::print("generating XACRO file:\t{}\n", xacro_filename);
    std::ofstream xacro_file;

    xacro_file.open(xacro_filename);

    xacro_file << "<?xml version='1.0'?>\n"
               << "<!-- This file has been generated automatically from create_xacro.cpp, do not edit by hand -->\n"
               << fmt::format("<robot xmlns:xacro='http://www.ros.org/wiki/xacro' name='{}'>\n", st_params::robot_name)
               << "<xacro:include filename='macro_definitions.urdf.xacro' />\n"
               << "<xacro:empty_link name='base_link'/>\n";

    std::string parent = "base_link";
    std::string child;
    for (int i = 0; i < st_params::num_segments; i++)
    {
        // create sections that gradually taper
        double segmentLength = st_params::lengths[2*i];
        double sectionLength = segmentLength / st_params::sections_per_segment;
        for (int j = 0; j < st_params::sections_per_segment; j++)
        {
            double sectionRadius = (st_params::diameters[i+1]/2 * j + st_params::diameters[i]/2 * (st_params::sections_per_segment-j))/st_params::sections_per_segment;
            double mass = st_params::totalMass / (st_params::num_segments * st_params::sections_per_segment); /** @todo fix to use value calculated from area of cross-section */
            child = fmt::format("seg{}_sec{}-{}_connect", i, j, j+1);
            if (j == st_params::sections_per_segment - 1) // for last section, child connects to next part outside segment
                child = fmt::format("seg{}-{}_connect", i, i+1);
            xacro_file << fmt::format("<xacro:PCC id='seg{}_sec{}' parent='{}' child='{}' length='{}' mass='{}' radius='{}'/>\n", i, j, parent, child, sectionLength, 0.2, sectionRadius);
            xacro_file << fmt::format("<xacro:empty_link name='{}'/>\n", child);
            parent = child;
        }
        /** @todo incorporate length of connector part */
    }
    xacro_file << "</robot>";

    xacro_file.close();
    fmt::print("XACRO file generated.\n");

    fmt::print("generating URDF file (ROS and XACRO must be installed):\t{}\n", urdf_filename);
    std::system(fmt::format("rosrun xacro xacro -o {} {}", urdf_filename, xacro_filename).c_str());
    fmt::print("URDF file generated.\n");
}