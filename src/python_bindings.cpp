#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <3d-soft-trunk/AugmentedRigidArm.h>
#include <3d-soft-trunk/CurvatureCalculator.h>
#include <3d-soft-trunk/SoftTrunkModel.h>
#include <3d-soft-trunk/ControllerPCC.h>
#include <3d-soft-trunk/OSC.h>

namespace py = pybind11;

PYBIND11_MODULE(softtrunk_pybind_module, m){
    /** @todo would be better if elements could be set like `state.q[0] = 0.1` in Python */
    py::class_<srl::State>(m, "State", "individual elements in array can be read but not set, see example codes")
    .def(py::init<>())
    .def(py::init<int>())
    .def("setSize", &srl::State::setSize)
    .def_property("q", [](srl::State& s){return s.q;}, [](srl::State& s, const VectorXd& q){s.q=q;})
    .def_property("dq", [](srl::State& s){return s.dq;}, [](srl::State& s, const VectorXd& dq){s.dq=dq;})
    .def_property("ddq", [](srl::State& s){return s.ddq;}, [](srl::State& s, const VectorXd& ddq){s.ddq=ddq;});

    /** @todo implement ability to edit parameters from Python interface as needed */
    py::class_<SoftTrunkParameters>(m, "SoftTrunkParameters")
    .def(py::init<>())
    .def("getBlankState", &SoftTrunkParameters::getBlankState)
    .def("finalize", &SoftTrunkParameters::finalize);

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

    py::class_<CurvatureCalculator> cc(m, "CurvatureCalculator");
    cc.def(py::init<SoftTrunkParameters, CurvatureCalculator::SensorType, std::string>())
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
    stm.def(py::init<SoftTrunkParameters>())
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


    py::class_<ControllerPCC>(m, "ControllerPCC")
    .def(py::init<SoftTrunkParameters, CurvatureCalculator::SensorType, bool>())
    .def("set_ref", py::overload_cast<const srl::State&>(&ControllerPCC::set_ref))
    .def("get_state", [](ControllerPCC& cpcc){
        srl::State state;
        cpcc.get_state(state);
        return state;
    })
    .def("get_pressure", &ControllerPCC::get_pressure)
    .def("set_state", &ControllerPCC::set_state)
    .def("toggle_log", &ControllerPCC::toggle_log)
    .def("simulate", &ControllerPCC::simulate)
    .def("set_frequency", &ControllerPCC::set_frequency);


    py::class_<OSC>(m, "OSC")
    .def(py::init<SoftTrunkParameters, CurvatureCalculator::SensorType, int>())
    .def("set_ref", py::overload_cast<const srl::State&>(&OSC::set_ref))
    .def("set_ref", py::overload_cast<const Vector3d, const Vector3d&, const Vector3d&>(&OSC::set_ref))
    .def("get_x", [](OSC& osc){
        Vector3d x;
        osc.get_x(x);
        return x;
    })
    .def("get_kp", &OSC::get_kp)
    .def("get_kd", &OSC::get_kd)
    .def("toggleGripper", &OSC::toggleGripper)
    .def_property("gripperAttached", [](OSC& osc){return osc.gripperAttached;}, [](OSC& osc, bool b){osc.gripperAttached = b;})
    .def_property("loadAttached", [](OSC& osc){return osc.loadAttached;}, [](OSC& osc, double d){osc.loadAttached = d;});

    py::enum_<CurvatureCalculator::SensorType>(cc, "SensorType")
    .value("qualisys", CurvatureCalculator::SensorType::qualisys)
    .value("bend_labs", CurvatureCalculator::SensorType::bend_labs)
    .value("simulator", CurvatureCalculator::SensorType::simulator);
}
