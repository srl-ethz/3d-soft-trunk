#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <3d-soft-trunk/VisualizerROS.h>

namespace py = pybind11;

/**
 * @brief ros::init has to be called before creating ros nodehandles. Since rospy.init called in Python doesn't seem to have an effect in the C++ libraries, this functionality is provided as a pybind function.
 * Yes, this is a super hacky solution.
 */
void ros_init_custom(std::string node_name){
    int argc = 0;
    char* argv[1] = {""};
    ros::init(argc, argv, node_name);
}

/**
 * @brief Python interface for classes that require ROS to be installed
 */
PYBIND11_MODULE(softtrunk_ROS_pybind_module, m){
    py::class_<VisualizerROS>(m, "VisualizerROS")
    .def(py::init<SoftTrunkModel&>())
    .def("publishState", &VisualizerROS::publishState)
    .def("publishArrow", &VisualizerROS::publishArrow);
    m.def("ros_init_custom", &ros_init_custom, "call ros::init inside C++");
    m.def("ros_ok_custom", [](){return ros::ok();}, "call ros::ok() inside C++");
}