#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <3d-soft-trunk/VisualizerROS.h>

namespace py = pybind11;

/**
 * @brief Python interface for classes that require ROS to be installed
 */
PYBIND11_MODULE(softtrunk_ROS_pybind_module, m){
    py::class_<VisualizerROS>(m, "VisualizerROS")
    .def(py::init<SoftTrunkModel&>())
    .def("publishState", &VisualizerROS::publishState)
    .def("publishArrow", &VisualizerROS::publishArrow);
}