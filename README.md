# 3D Soft Trunk repository
## [SoPrA fabrication guide](https://gitlab.ethz.ch/srl/3d-soft-trunk/-/wikis/home)
available in the wiki of this repository

## get this repository

In the desired directory, run...
```bash
## option 1: clone with https- enter username & password each time you access remote
git clone --recursive https://gitlab.ethz.ch/srl/3d-soft-trunk.git 
```
or
```bash
## option 2: clone with SSH- need to set up SSH key in GitLab, no username / password required
git clone --recursive git@gitlab.ethz.ch:srl/3d-soft-trunk.git
```
(`--recursive` option will automatically clone the submodules as well)

## install necessary packages for mobilerack-interface

**refer to [README of mobilerack-interface](mobilerack-interface/README.md)**

also install these packages:
```bash
sudo apt install python3-pip
pip3 install xacro # used to convert robot model files from XACRO to URDF
```

## install Drake

refer to [Drake documentation- binary installation](https://drake.mit.edu/from_binary.html) and [Drake sample CMake project](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed).

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

executables are output to bin, libraries are output to lib/. For visualization of the model, use the Drake Visualizer, at `/opt/drake/bin/drake-visualizer`.

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