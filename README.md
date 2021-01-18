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

For Ubuntu 18.04, basic steps are:
```bash
curl -O https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-bionic.tar.gz
## decompress and place drake files into /opt/drake
sudo tar -xvzf drake-latest-bionic.tar.gz -C /opt
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

executables are output to bin/.

## Generating Documentation

Uses Doxygen to generate documentation from inline comments in code. Install [Doxygen](http://www.doxygen.nl), and
run `doxygen` in this directory to generate HTML (can be seen with browser at html/index.html) & LATEX output.

# Fabrication of Physical Soft Robot
See **Soft Robot Fabrication.md**

# Bill of materials
See **Bill of materials.csv**
