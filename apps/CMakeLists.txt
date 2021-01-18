
### examples
add_executable(example_CurvatureCalculator example_CurvatureCalculator.cpp)
target_link_libraries(example_CurvatureCalculator CurvatureCalculator)

add_executable(example_AugmentedRigidArm example_AugmentedRigidArm.cpp)
target_link_libraries(example_AugmentedRigidArm AugmentedRigidArm)

add_executable(example_realtime_visualize example_realtime_visualize.cpp)
target_link_libraries(example_realtime_visualize AugmentedRigidArm CurvatureCalculator)

### other programs (experiments etc.)
add_executable(create_urdf create_urdf.cpp)
target_link_libraries(create_urdf fmt)

add_executable(characterize_alpha characterize_alpha.cpp)
target_link_libraries(characterize_alpha ControllerPCC)

add_executable(experiment_pid experiment_pid.cpp)
target_link_libraries(experiment_pid ControllerPCC)

# add_executable(experiment src/experiment.cpp)
# target_link_libraries(experiment Manager -pthread)