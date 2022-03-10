#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <3d-soft-trunk/Models/AugmentedRigidArm.h>
#include <3d-soft-trunk/StateEstimator.h>
#include <3d-soft-trunk/Models/SoftTrunkModel.h>
#include <3d-soft-trunk/ControllerPCC.h>

namespace py = pybind11;

PYBIND11_MODULE(softtrunk_pybind_module, m){
    /** @todo would be better if elements could be set like `state.q[0] = 0.1` in Python */
    py::class_<srl::State>(m, "State", "individual elements in array can be read but not set, see example codes")
    .def(py::init<>())
    .def("setSize", &srl::State::setSize)
    .def_property("q", [](srl::State& s){return s.q;}, [](srl::State& s, const VectorXd& q){s.q=q;})
    .def_property("dq", [](srl::State& s){return s.dq;}, [](srl::State& s, const VectorXd& dq){s.dq=dq;})
    .def_property("ddq", [](srl::State& s){return s.ddq;}, [](srl::State& s, const VectorXd& ddq){s.ddq=ddq;});

    /** @todo implement ability to edit parameters from Python interface as needed */
    py::class_<SoftTrunkParameters>(m, "SoftTrunkParameters")
    .def(py::init<>())
    .def("getBlankState", &SoftTrunkParameters::getBlankState)
    .def("finalize", &SoftTrunkParameters::finalize)
    .def("load_yaml", &SoftTrunkParameters::load_yaml);

    py::class_<AugmentedRigidArm>(m, "AugmentedRigidArm")
    .def(py::init<SoftTrunkParameters>())
    .def("update", &AugmentedRigidArm::update)
    .def("get_H", [](AugmentedRigidArm& ara, int i){
        return ara.get_H(i).matrix();
    })
    .def("get_H_tip", [](AugmentedRigidArm& ara){
        return ara.get_H_tip().matrix();
    })
    .def("get_H_base", [](AugmentedRigidArm& ara){
        return ara.get_H_base().matrix();
    });

    py::class_<StateEstimator> ste(m, "StateEstimator");
    ste.def(py::init<SoftTrunkParameters>())
    .def("poll_sensors", &StateEstimator::poll_sensors);
    
    py::class_<SoftTrunkModel> stm(m, "SoftTrunkModel");
    stm.def(py::init<SoftTrunkParameters>())
    .def("set_state", &SoftTrunkModel::set_state)
    .def("getModel", [](SoftTrunkModel& stm){
        return std::make_tuple(stm.dyn_.B, stm.dyn_.c, stm.dyn_.g, stm.dyn_.K, stm.dyn_.D, stm.dyn_.A, stm.dyn_.J);
    })
    .def("get_H", [](SoftTrunkModel& stm, int i){
        return stm.get_H(i).matrix();
    })
    .def("get_H_base", [](SoftTrunkModel& stm){
        return stm.get_H_base().matrix();
    });


    py::class_<ControllerPCC>(m, "ControllerPCC")
    .def(py::init<SoftTrunkParameters>())
    .def("set_ref", py::overload_cast<const srl::State&>(&ControllerPCC::set_ref))
    .def("toggle_log", &ControllerPCC::toggle_log)
    .def("simulate", &ControllerPCC::simulate);
}
