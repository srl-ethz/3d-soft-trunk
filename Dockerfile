FROM robotlocomotion/drake:latest
# skip interactive setup when installing packages
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update --yes; apt-get install --yes build-essential

# upgrade cmake
RUN apt-get remove --yes --purge cmake
RUN apt-get update --yes; apt-get install --yes libssl-dev wget
RUN wget https://github.com/Kitware/CMake/releases/download/v3.16.5/cmake-3.16.5.tar.gz
RUN tar -zxvf cmake-3.16.5.tar.gz
RUN cd cmake-3.16.5; ./bootstrap; make; make install

RUN apt-get update --yes; apt-get install --yes python3-dev python3-numpy
RUN apt-get update --yes; apt-get install --yes libmodbus-dev libeigen3-dev libserialport-dev libopencv-dev