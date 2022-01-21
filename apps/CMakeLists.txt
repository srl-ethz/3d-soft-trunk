### examples
add_executable(example_CurvatureCalculator example_CurvatureCalculator.cpp)
target_link_libraries(example_CurvatureCalculator CurvatureCalculator)

add_executable(example_AugmentedRigidArm example_AugmentedRigidArm.cpp)
target_link_libraries(example_AugmentedRigidArm AugmentedRigidArm)

add_executable(example_realtime_visualize example_realtime_visualize.cpp)
target_link_libraries(example_realtime_visualize AugmentedRigidArm CurvatureCalculator)

add_executable(example_SoftTrunkModel example_SoftTrunkModel.cpp)
target_link_libraries(example_SoftTrunkModel SoftTrunkModel)

add_executable(example_Simulator example_Simulator.cpp)
target_link_libraries(example_Simulator ControllerPCC)

add_executable(example_Characterizer example_Characterizer.cpp)
target_link_libraries(example_Characterizer Characterize)

add_executable(example_dynamic example_dynamic.cpp)
target_link_libraries(example_dynamic Dyn)

add_executable(example_IDCon example_IDCon.cpp)
target_link_libraries(example_IDCon IDCon)

if(${roscpp_FOUND})
    add_executable(example_VisualizerROS example_VisualizerROS.cpp)
    target_link_libraries(example_VisualizerROS SoftTrunkModel VisualizerROS)
endif(${roscpp_FOUND})



### other programs (experiments etc.)

add_executable(example_Controller example_Controller.cpp)
target_link_libraries(example_Controller ControllerPCC OSC)

add_executable(characterization_actuation characterization_actuation.cpp)
target_link_libraries(characterization_actuation ValveController SoftTrunkModel CurvatureCalculator)

add_executable(simulate_logged_pressure simulate_logged_pressure.cpp)
target_link_libraries(simulate_logged_pressure ControllerPCC Threads::Threads)

add_executable(record_movement record_movement.cpp)
target_link_libraries(record_movement SerialInterface ControllerPCC)

add_executable(play_movement play_movement.cpp)
target_link_libraries(play_movement SerialInterface ControllerPCC)

add_executable(example_QuasiStatic example_QuasiStatic.cpp)
target_link_libraries(example_QuasiStatic QuasiStatic)

add_executable(prismatic_datagathering prismatic_datagathering.cpp)
target_link_libraries(prismatic_datagathering ValveController QualisysClient)

add_executable(prismatic_controller prismatic_controller.cpp)
target_link_libraries(prismatic_controller ValveController QualisysClient)

add_executable(spline_parser spline_parser.cpp)
target_link_libraries(spline_parser ${EIGEN3_LIBRARIES} fmt)

if(${roscpp_FOUND})
    add_executable(ui_controller ui_controller.cpp)
    target_link_libraries(ui_controller SoftTrunkModel OSC VisualizerROS)
endif(${roscpp_FOUND})
