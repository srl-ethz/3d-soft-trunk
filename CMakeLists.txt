cmake_minimum_required (VERSION 3.10.2)
project(SoftTrunk)

add_subdirectory(mobilerack-interface)

### find and setup for Drake
# cf: https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed
# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

# include(CTest)

if(APPLE)
  set(FIND_PYTHON_EXECUTABLE_PATHS /usr/local/opt/python@3.8/bin)
  set(FIND_PYTHON_INTERP_VERSION 3.8)
else()
  set(FIND_PYTHON_EXECUTABLE_PATHS /usr/bin)
  set(FIND_PYTHON_INTERP_VERSION 3.6)
endif()
find_program(PYTHON_EXECUTABLE NAMES python3
  PATHS "${FIND_PYTHON_EXECUTABLE_PATHS}"
  NO_DEFAULT_PATH
)
find_package(PythonInterp ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

execute_process(COMMAND ${PYTHON_EXECUTABLE}-config --exec-prefix
  OUTPUT_VARIABLE PYTHON_EXEC_PREFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
list(APPEND CMAKE_PREFIX_PATH "${PYTHON_EXEC_PREFIX}")
find_package(PythonLibs ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

find_package(drake CONFIG REQUIRED)

get_filename_component(PYTHONPATH
  "${drake_DIR}/../../python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages"
  REALPATH
)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

### END OF find and setup for Drake

# setup output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(include mobilerack-interface/include mobilerack-interface/subm/MiniPID)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

ADD_LIBRARY(CurvatureCalculator SHARED src/CurvatureCalculator.cpp)
TARGET_LINK_LIBRARIES(CurvatureCalculator ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} QualisysClient fmt)

# add_library(Manager SHARED src/Manager.cpp)
# target_link_libraries(Manager SoftTrunkInterface AugmentedRigidArm ControllerPCC ${EIGEN3_LIBRARIES} -pthread)

add_executable(example_CurvatureCalculator src/example_CurvatureCalculator.cpp)
target_link_libraries(example_CurvatureCalculator CurvatureCalculator)

add_library(AugmentedRigidArm SHARED src/AugmentedRigidArm.cpp)
target_link_libraries(AugmentedRigidArm drake::drake ${catkin_LIBRARIES})

add_executable(create_urdf src/create_urdf.cpp)
target_link_libraries(create_urdf fmt)

add_executable(example_AugmentedRigidArm src/example_AugmentedRigidArm.cpp)
target_link_libraries(example_AugmentedRigidArm AugmentedRigidArm)

add_executable(example_realtime_visualize src/example_realtime_visualize.cpp)
target_link_libraries(example_realtime_visualize AugmentedRigidArm CurvatureCalculator)

add_library(ControllerPCC SHARED src/ControllerPCC.cpp mobilerack-interface/subm/MiniPID/MiniPID.cpp)
target_link_libraries(ControllerPCC ${EIGEN3_LIBRARIES} AugmentedRigidArm CurvatureCalculator ValveController -pthread)

# add_executable(example_ControllerPCC src/example_ControllerPCC.cpp)
# target_link_libraries(example_ControllerPCC AugmentedRigidArm ControllerPCC SoftTrunkInterface)

# add_executable(characterize_part2 src/characterize_part2.cpp)
# target_link_libraries(characterize_part2 Manager)

# add_executable(characterize_part1 src/characterize_part1.cpp)
# target_link_libraries(characterize_part1 Manager)

# add_executable(experiment src/experiment.cpp)
# target_link_libraries(experiment Manager -pthread)

add_executable(experiment_pid src/experiment_pid.cpp)
target_link_libraries(experiment_pid ControllerPCC)