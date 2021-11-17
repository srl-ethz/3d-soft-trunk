
rm /etc/apt/sources.list.d/kitware.list
rm /etc/apt/sources.list.d/kitware.list.save

apt-get update

apt-get install libopencv-dev
apt-get install libmodbus-dev
apt install libserialport-dev
apt install ros-noetic-desktop-full

mkdir -p build
cd build
cmake -DCMAKE_PREFIX_PATH="/opt/drake;/opt/ros/noetic" ..
make

echo 'export PYTHONPATH=$PYTHONPATH:/src/3d-soft-trunk/lib' >> ~/.bashrc
