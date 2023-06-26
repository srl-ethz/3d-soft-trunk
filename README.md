# 3D Soft Trunk repository
## [SoPrA fabrication guide](https://github.com/srl-ethz/3d-soft-trunk/wiki)
check the Wiki of this repo for details of fabrication process, links to STL files, etc.

## Installing this repository

Set up SSH key authentication for GitHub, then in the desired directory, run...
```bash
git clone --recursive git@github.com:srl-ethz/3d-soft-trunk.git
```
(`--recursive` option will automatically clone the submodules as well)

## Install necessary packages for mobilerack-interface

**Refer to [README of mobilerack-interface](mobilerack-interface/README.md)**

Also install these packages:
```bash
sudo apt install python3-pip
pip3 install xacro # used to convert robot model files from XACRO to URDF
```

## Install Drake

Refer to [Drake documentation- binary installation](https://drake.mit.edu/from_binary.html) and [Drake sample CMake project](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed).

For Ubuntu 20.04, basic steps are:
```bash
curl -O https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-focal.tar.gz
## decompress and place drake files into /opt/drake
sudo tar -xvzf drake-latest-focal.tar.gz -C /opt
## install prerequisites
sudo /opt/drake/share/drake/setup/install_prereqs
```

For macOS, .... TODO

## Compile

```bash
cd /path/to/3d-soft-trunk
cmake -DCMAKE_PREFIX_PATH=/opt/drake .
make
```

Executables are output to bin, libraries are output to lib/.
For visualization of the model, use the Meshcat Visualizer (Drake visualizer [has](https://stackoverflow.com/questions/75303201/drake-meshcat-visualizer-example-on-ubuntu-22-04-with-apt-installation) [been](https://github.com/RobotLocomotion/drake/issues/) deprecated).
Launch `/opt/drake/bin/meldis` (which [relays LCM connections from Drake to the Meshcat visualization](https://drake.mit.edu/pydrake/pydrake.visualization.meldis.html)), and go to the URL shown (probably http://localhost:7000) in your browser.

## Python interface
In its current implementation, you must set the `$PYTHONPATH` environment variable to point to the directory containing the library binaries in order to run. (probably `3d-soft-trunk/lib`)

```bash
## run this everytime you open a new terminal to run a python script using this library
export PYTHONPATH=$PYTHONPATH:/path/to/lib
## Alternatively, append the line to ~/.bashrc if you don't want to run it every time.
python3
>> import mobilerack_pybind_module
>> vc = mobilerack_pybind_module.ValveController("192.168.0.100", [0, 1], 200)
>> vc.setSinglePressure(0, 100)

>> import softtrunk_pybind_module
>> st_params = softtrunk_pybind_module.SoftTrunkParameters()
>> st_params.finalize()
>> stm = softtrunk_pybind_module.SoftTrunkModel(st_params)
>> stm.getModel()
```

see more examples in `examples_python/` and `mobilerack-interface/examples_python`.

## Generating Documentation

Uses Doxygen to generate documentation from inline comments in code. Install [Doxygen](http://www.doxygen.nl), and
run `doxygen Doxyfile` in this directory to generate HTML (can be seen with browser at html/index.html) & LATEX output.

Example code are in `apps/example_*.cpp` and `examples_python/example_*.py`.

# I want to add my own sensor/controller/model

Refer to the README files in `src/`.

# I am not a member of SRL
You will likely be using different sensors and actuators. Refer to the wiki for how to add new sensors to the `StateEstimator`.
To add your own actuation method, you will need to add a submodule class which allows you to set inputs for the actuators, and replace the `ValveController` object in the `ControllerPCC` class.
Then you simply need to adjust the `ControllerPCC::actuate(VectorXd p)` to use your new actuator instead, and the framework should work.
