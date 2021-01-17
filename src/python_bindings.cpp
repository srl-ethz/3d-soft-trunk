#include <pybind11/pybind11.h>

#include <AugmentedRigidArm.h>

namespace py = pybind11;

PYBIND11_MODULE(softtrunk_pybind_module, m){
    py::class_<AugmentedRigidArm>(m, "AugmentedRigidArm")
    .def(py::init<>())
    .def("update", &AugmentedRigidArm::update);
}