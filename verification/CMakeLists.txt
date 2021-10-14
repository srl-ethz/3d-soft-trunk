add_executable(stateestimator stateestimator.cpp)
target_link_libraries(stateestimator StateEstimator)

add_executable(model model.cpp)
target_link_libraries(model Model)