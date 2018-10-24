# Fabrication of Physical Soft Robot
See **Soft Robot Fabrication.md**

# Installing necessary libraries

## libmodbus
Communication to FESTO valve array via mod bus 
```
sudo apt install libmodbus-dev
```

## NatNetLinux
This linux library is needed for listening to a udp communication from Optitrack Motive 1.10.0 on a windows machine and streaming rigid bodies. Ensure you install the [Prerequisites](https://github.com/rocketman768/NatNetLinux) for NatNetLinux.
In the /3rd directory, NatNetLinux is added as a submodule. After fetching the submodule with:
```
git submodule init
git submodule update
```
compile and install NatNetLinux:
```
mkdir build
cd build
cmake ../NatNetLinux
make
sudo make install
```
## Eigen3
[Get the code](http://eigen.tuxfamily.org/index.php?title=Main_Page), unzip, navigate into the unzipped folder, prepare with cmake and then install this header libary:
```
mkdir build
cd build/
cmake ..
sudo make install
```

## RBDL
This code uses the [Rigid Body Dynamics Library](https://rbdl.bitbucket.io/index.html) for calculating the dynamics of the rigid bodies of the augmented robot model.
Download the most recent stable version as zip file, then follow its README to install, but make sure to also compile the URDF reader addon, see below:
```
mkdir build
cd build/
cmake -D RBDL_BUILD_ADDON_URDFREADER=ON CMAKE_BUILD_TYPE=Release ../
make
sudo make install
```
You need to install Eigen3 before, as described above.

# Libraries
each library usually has a demo program, just compile the library with `cmake .; make`.
## OptiTrackClient.cpp
Talks to Motive software over UDP, receives current information for each RigidBody which can then be used in CurvatureCalculator.cpp (see below).

## CurvatureCalculator.cpp
Calculates the curvature of each soft arm segment based on the OptiTrack measurements of the base and tip of a segment. This has not been tested on the actual system yet.

example_CurvatureCalculator.cpp is the sample code using this library - This demo prints out for each segment of the soft arm the calculated degree of curvature called theta and the rotation around the center axis called phi.

## forceController.cpp
Implements an individual PID control for each valve of the FESTO valve array.

example_sinusoidal.cpp and example_forceController.cpp is a demo of this library. The former sends out sinusoidal signals to each compartment.

## arm.cpp
Supposed to consolidate all the kinematic & dynamic info about the arm, but still a work in progress.

# programs
(only libraries and demo for each library has been created, no programs that combine the libraries yet.)


# comments in code
As suggested [here](https://softwareengineering.stackexchange.com/questions/84071/is-it-better-to-document-functions-in-the-header-file-or-the-source-file),
* **how to use** the function / class / variable will be commented in the header file
* **how the code works** will be commented in the source code