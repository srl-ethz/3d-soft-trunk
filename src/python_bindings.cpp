#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <3d-soft-trunk/AugmentedRigidArm.h>
#include <3d-soft-trunk/CurvatureCalculator.h>

namespace py = pybind11;

PYBIND11_MODULE(softtrunk_pybind_module, m){
    py::class_<AugmentedRigidArm>(m, "AugmentedRigidArm")
    .def(py::init<>())
    .def("update", &AugmentedRigidArm::update);
    

    py::class_<CurvatureCalculator> cc(m, "CurvatureCalculator");
    cc.def(py::init<CurvatureCalculator::SensorType>())
    .def("get_curvature", [](CurvatureCalculator& cc){
        // return as tuple rather than by reference
        VectorXd q; VectorXd dq; VectorXd ddq;
        cc.get_curvature(q, dq, ddq);
        return std::make_tuple(q, dq, ddq);
    });

    py::enum_<CurvatureCalculator::SensorType>(cc, "SensorType")
    .value("qualisys", CurvatureCalculator::SensorType::qualisys)
    .value("bend_labs", CurvatureCalculator::SensorType::bend_labs);
}