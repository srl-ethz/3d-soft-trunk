
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
target_link_libraries(example_Simulator SoftTrunkModel Simulator)

if(${roscpp_FOUND})
    add_executable(example_VisualizerROS example_VisualizerROS.cpp)
    target_link_libraries(example_VisualizerROS SoftTrunkModel VisualizerROS)
    target_include_directories(example_VisualizerROS PRIVATE ${roscpp_INCLUDE_DIRS})    
endif(${roscpp_FOUND})


### other programs (experiments etc.)

add_executable(experiment_pid experiment_pid.cpp)
target_link_libraries(experiment_pid ControllerPCC)

add_executable(characterization_actuation characterization_actuation.cpp)
target_link_libraries(characterization_actuation ValveController SoftTrunkModel CurvatureCalculator)

add_executable(simulate_logged_pressure simulate_logged_pressure.cpp)
target_link_libraries(simulate_logged_pressure Simulator SoftTrunkModel Threads::Threads)