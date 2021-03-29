#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <3d-soft-trunk/AugmentedRigidArm.h>
#include <3d-soft-trunk/CurvatureCalculator.h>
#include <3d-soft-trunk/SoftTrunkModel.h>
#include <3d-soft-trunk/Simulator.h>

namespace py = pybind11;

PYBIND11_MODULE(softtrunk_pybind_module, m){
    /** @todo would be better if elements could be set like `state.q[0] = 0.1` in Python */
    py::class_<srl::State>(m, "State", "individual elements in array can be read but not set, see example codes")
    .def(py::init<>())
    .def_property("q", [](srl::State& s){return s.q;}, [](srl::State& s, const VectorXd& q){s.q=q;})
    .def_property("dq", [](srl::State& s){return s.dq;}, [](srl::State& s, const VectorXd& dq){s.dq=dq;})
    .def_property("ddq", [](srl::State& s){return s.ddq;}, [](srl::State& s, const VectorXd& ddq){s.ddq=ddq;});

    py::class_<AugmentedRigidArm>(m, "AugmentedRigidArm")
    .def(py::init<>())
    .def("update", &AugmentedRigidArm::update)
    .def("get_H", [](AugmentedRigidArm& ara, int i){
        return ara.get_H(i).matrix();
    })
    .def("get_H_tip", [](AugmentedRigidArm& ara){
        return ara.get_H_tip().matrix();
    });

    py::class_<CurvatureCalculator> cc(m, "CurvatureCalculator");
    cc.def(py::init<CurvatureCalculator::SensorType, std::string>())
    .def("get_curvature", [](CurvatureCalculator& cc){
        // return as tuple rather than by reference
        srl::State state;
        cc.get_curvature(state);
        return std::make_tuple(state.q, state.dq, state.ddq);
    })
    .def("get_frame", [](CurvatureCalculator& cc, int i){
        Eigen::Matrix<double, 4, 4> H = Eigen::Matrix<double, 4, 4>::Identity();
        H = cc.get_frame(i).matrix();
        return H;
    })
    .def("get_timestamp", &CurvatureCalculator::get_timestamp);
    
    py::class_<SoftTrunkModel> stm(m, "SoftTrunkModel");
    stm.def(py::init<>())
    .def("updateState", &SoftTrunkModel::updateState)
    .def("getModel", [](SoftTrunkModel& stm){
        return std::make_tuple(stm.B, stm.c, stm.g, stm.K, stm.D, stm.A, stm.J);
    })
    .def("get_H", [](SoftTrunkModel& stm, int i){
        return stm.get_H(i).matrix();
    })
    .def("get_H_base", [](SoftTrunkModel& stm){
        return stm.get_H_base().matrix();
    });

    py::class_<Simulator>(m, "Simulator")
    .def(py::init<SoftTrunkModel&, double, int, srl::State&>())
    .def("simulate", &Simulator::simulate)
    .def("getState", [](Simulator& sim){
        srl::State state;
        sim.get_state(state);
        return state;
    })
    .def("start_log", &Simulator::start_log)
    .def("end_log", &Simulator::end_log);

    py::enum_<CurvatureCalculator::SensorType>(cc, "SensorType")
    .value("qualisys", CurvatureCalculator::SensorType::qualisys)
    .value("bend_labs", CurvatureCalculator::SensorType::bend_labs);
}
