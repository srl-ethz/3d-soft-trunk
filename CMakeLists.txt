cmake_minimum_required (VERSION 3.12)
project(SoftTrunk)

add_subdirectory(mobilerack-interface)

### find and setup for Drake
# cf: https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed
# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

# include(CTest)

## commented out these lines setting up Python for Drake, since the setting of Python executable was interfering with this repo's pybind11
## so far commenting out doesn't seem to cause a problem
# if(APPLE)
#   set(FIND_PYTHON_EXECUTABLE_PATHS /usr/local/opt/python@3.8/bin)
#   set(FIND_PYTHON_INTERP_VERSION 3.8)
# else()
#   set(FIND_PYTHON_EXECUTABLE_PATHS /usr/bin)
#   set(FIND_PYTHON_INTERP_VERSION 3.6)
# endif()
# find_program(PYTHON_EXECUTABLE NAMES python3
#   PATHS "${FIND_PYTHON_EXECUTABLE_PATHS}"
#   NO_DEFAULT_PATH
# )
# find_package(PythonInterp ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

# execute_process(COMMAND ${PYTHON_EXECUTABLE}-config --exec-prefix
#   OUTPUT_VARIABLE PYTHON_EXEC_PREFIX
#   OUTPUT_STRIP_TRAILING_WHITESPACE
# )
# list(APPEND CMAKE_PREFIX_PATH "${PYTHON_EXEC_PREFIX}")
# find_package(PythonLibs ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

find_package(drake CONFIG REQUIRED)

# get_filename_component(PYTHONPATH
#   "${drake_DIR}/../../python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages"
#   REALPATH
# )

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

### END OF find and setup for Drake

# setup output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# use this when referring to files in project from C++ source
add_definitions(-DSOFTTRUNK_PROJECT_DIR="${PROJECT_SOURCE_DIR}")

include_directories(include mobilerack-interface/include)
add_subdirectory(subm/yaml-cpp)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(roscpp) # this is optionally required for VisualizerROS

### libraries
ADD_LIBRARY(CurvatureCalculator SHARED src/Sensors/CurvatureCalculator.cpp)
TARGET_LINK_LIBRARIES(CurvatureCalculator ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} QualisysClient SerialInterface fmt yaml-cpp)

ADD_LIBRARY(MotionCapture SHARED src/Sensors/MotionCapture.cpp)
TARGET_LINK_LIBRARIES(MotionCapture ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} QualisysClient fmt yaml-cpp)

ADD_LIBRARY(BendLabs SHARED src/Sensors/BendLabs.cpp)
TARGET_LINK_LIBRARIES(BendLabs ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} SerialInterface fmt yaml-cpp)

ADD_LIBRARY(StateEstimator SHARED src/StateEstimator.cpp)
TARGET_LINK_LIBRARIES(StateEstimator BendLabs MotionCapture)

add_library(AugmentedRigidArm SHARED src/Models/AugmentedRigidArm.cpp)
target_link_libraries(AugmentedRigidArm drake::drake yaml-cpp)

add_library(SoftTrunkModel SHARED src/Models/SoftTrunkModel.cpp)
target_link_libraries(SoftTrunkModel AugmentedRigidArm)

add_library(Lagrange SHARED src/Models/Lagrange.cpp)
target_link_libraries(Lagrange yaml-cpp fmt)

add_library(Model SHARED src/Model.cpp)
target_link_libraries(Model StateEstimator SoftTrunkModel Lagrange)

add_library(ControllerPCC SHARED src/ControllerPCC.cpp)
target_link_libraries(ControllerPCC Model StateEstimator Threads::Threads yaml-cpp)

add_library(OSC SHARED src/Controllers/OSC.cpp)
target_link_libraries(OSC ControllerPCC)

add_library(PID SHARED src/Controllers/PID.cpp)
target_link_libraries(PID ControllerPCC MiniPID) 

add_library(LQR SHARED src/Controllers/LQR.cpp)
target_link_libraries(LQR ControllerPCC)

add_library(Dyn src/Controllers/Dyn.cpp)
target_link_libraries(Dyn ControllerPCC)

add_library(Characterize src/Controllers/Characterize.cpp)
target_link_libraries(Characterize ControllerPCC)

add_library(IDCon src/Controllers/IDCon.cpp)
target_link_libraries(IDCon ControllerPCC)

add_library(QuasiStatic src/Controllers/QuasiStatic.cpp)
target_link_libraries(QuasiStatic ControllerPCC)


if(${roscpp_FOUND})
#    add_library(VisualizerROS src/VisualizerROS.cpp)
#    target_link_libraries(VisualizerROS SoftTrunkModel ${roscpp_LIBRARIES})
#    target_include_directories(VisualizerROS PUBLIC ${roscpp_INCLUDE_DIRS})    
endif(${roscpp_FOUND})

### set up pybind modules
pybind11_add_module(softtrunk_pybind_module src/python_bindings.cpp)
target_link_libraries(softtrunk_pybind_module PUBLIC AugmentedRigidArm CurvatureCalculator SoftTrunkModel ControllerPCC)

if(${roscpp_FOUND})
    # pybind modules for classes that require ROS
#    pybind11_add_module(softtrunk_ROS_pybind_module src/python_bindings_ros.cpp)
#    target_link_libraries(softtrunk_ROS_pybind_module PUBLIC VisualizerROS ${roscpp_LIBRARIES})
endif(${roscpp_FOUND})

add_subdirectory(apps)
add_subdirectory(verification)
add_subdirectory(experiments_IROS2021)
add_subdirectory(experiments_ICRA2021)