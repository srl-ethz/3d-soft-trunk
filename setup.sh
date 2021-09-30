apt-get install libopencv-dev
apt-get install libmodbus-dev
apt install libserialport-dev
apt install ros-noetic-desktop-full
cmake -DCMAKE_PREFIX_PATH="/opt/drake;/opt/ros/noetic" .
make