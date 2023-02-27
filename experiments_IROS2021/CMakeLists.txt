add_executable(record_qtm_frames record_qtm_frames.cpp)
target_link_libraries(record_qtm_frames QualisysClient yaml-cpp)

add_executable(sensor_visualize sensor_visualize.cpp)
target_link_libraries(sensor_visualize SoftTrunkModel SerialInterface)

if(${roscpp_FOUND})
#    add_executable(solve_tip_force solve_tip_force.cpp)
#    target_link_libraries(solve_tip_force SoftTrunkModel VisualizerROS SerialInterface ValveController drake::drake)
#    target_include_directories(solve_tip_force PRIVATE ${roscpp_INCLUDE_DIRS})    
endif(${roscpp_FOUND})
