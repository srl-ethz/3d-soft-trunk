# 3D Soft Trunk repository

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

refer to mobilerack-interface/README.md

## install Drake

refer to [Drake documentation- binary installation](https://drake.mit.edu/from_binary.html) and [Drake sample CMake project](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed).

For Ubuntu, basic steps are:
```bash
## after downloading & decompressing the binaries
cd /path/to/drake
sudo ./share/drake/setup/install_prereqs
mv /path/to/drake /opt/ ## move to /opt directory
```

For macOS, .... TODO

## Compile

```bash
cd /path/to/3d-soft-trunk
cmake -DCMAKE_PREFIX_PATH=/opt/drake .
make
```

executables are output to bin, libraries are output to lib/.

## Python interface
In its current implementation, you must set the `$PYTHONPATH` environment variable to point to the directory containing the library binaries in order to run. (probably `3d-soft-trunk/lib`)

```bash
## run this everytime you open a new terminal to run a python script using this library
PYTHONPATH=$PYTHONPATH:/path/to/lib
## Alternatively, append the line to ~/.bashrc if you don't want to run it every time.
python3
>> import mobilerack_pybind_module
>> vc = mobilerack_pybind_module.ValveController("192.168.0.100", [0, 1], 200)
>> vc.setSinglePressure(0, 100)

>> import softtrunk_pybind_module
>> aar = softtrunk_pybind_module.AugmentedRigidArm()
>> aar.update([0]*6, [0]*6)
```

see more examples in `examples_python/` and `mobilerack-interface/examples_python`.

## Generating Documentation

Uses Doxygen to generate documentation from inline comments in code. Install [Doxygen](http://www.doxygen.nl), and
run `doxygen` in this directory to generate HTML (can be seen with browser at html/index.html) & LATEX output.

# Fabrication of Physical Soft Robot
See **Soft Robot Fabrication.md**

# Bill of materials
See **Bill of materials.csv**
