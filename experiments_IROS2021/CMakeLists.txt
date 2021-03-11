add_executable(record_qtm_frames record_qtm_frames.cpp)
target_link_libraries(record_qtm_frames QualisysClient)

add_executable(cyclic_simulation cyclic_simulation.cpp)
target_link_libraries(cyclic_simulation SoftTrunkModel Threads::Threads)

add_executable(sensor_visualize sensor_visualize.cpp)
target_link_libraries(sensor_visualize SoftTrunkModel SerialInterface)

# I'm sorry this is a HACK!
find_package(roscpp)

if(${roscpp_FOUND})
    add_executable(solve_tip_force solve_tip_force.cpp)
    target_link_libraries(solve_tip_force SoftTrunkModel SerialInterface ValveController drake::drake ${roscpp_LIBRARIES})
    target_include_directories(solve_tip_force PRIVATE ${roscpp_INCLUDE_DIRS})    
endif(${roscpp_FOUND})
