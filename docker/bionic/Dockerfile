FROM robocomp/robocomp:bionic_fcl

# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp with DSR support in Ubuntu Bionic"

ARG DEBIAN_FRONTEND=noninteractive

RUN  echo "robolab:robolab" | chpasswd && adduser robolab sudo
ENV ROBOCOMP /home/robolab/robocomp/

### DSR package Dependencies
#RUN apt install -y software-properties-common
#RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt update
RUN apt install -y curl libasio-dev libtinyxml2-dev  libopencv-dev libqglviewer-dev-qt5 libeigen3-dev net-tools vim x11-utils iputils-ping netcat wget python3-dev python3-pybind11 gnupg2 software-properties-common
RUN python3 -m pip install pyrep
RUN add-apt-repository ppa:ubuntu-toolchain-r/test
### Third party dependencies
RUN curl -S https://raw.githubusercontent.com/robocomp/robocomp/development/classes/dsr/dependencies.sh | bash -x
RUN git clone https://github.com/ryanhaining/cppitertools /usr/local/include/cppitertools
RUN git clone https://github.com/pybind/pybind11
RUN cd ~/pybind11 && mkdir build && cd build && cmake .. && make -j$(nproc) && make install

# Last version of g++ gcc and cmake
RUN apt install -y cmake gcc-10 g++-10
ADD https://cmake.org/files/v3.19/cmake-3.19.0-Linux-x86_64.sh /cmake-3.19.0-Linux-x86_64.sh
RUN sudo mkdir /opt/cmake
RUN sudo sh /cmake-3.19.0-Linux-x86_64.sh --prefix=/opt/cmake --skip-license
RUN sudo ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake
# RUN update-alternatives --auto gcc 
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 1
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 1

#Compilation of DSR libs in Robocomp
USER robolab
RUN cd ~/robocomp/classes/dsr && git pull && mkdir build && cd build && cmake .. && make -j$(nproc)
USER root
ENV PATH=$PATH
RUN cd /home/robolab/robocomp/classes/dsr/build && make install
RUN ldconfig

#Compilation of DSR components
USER robolab
WORKDIR /home/robolab/robocomp/components/
RUN git clone --progress https://github.com/robocomp/dsr-graph.git 
WORKDIR /home/robolab/robocomp/components/dsr-graph/components/
RUN sed -i 's/opencv4/opencv3/g' viriatoDSR/src/CMakeListsSpecific.txt && sed -i 's/opencv4/opencv3/g' change_graph/src/CMakeListsSpecific.txt &&mkdir build && cd build && cmake .. && make -j$(nproc)
WORKDIR /home/robolab/robocomp/components/dsr-graph/

# Installation of Coppelia Sim
RUN sudo apt install xz-utils
RUN mkdir extras && cd extras && wget -c https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz -O - | sudo tar -xv -J
ENV COPPELIASIM_ROOT=/home/robolab/robocomp/components/dsr-graph/extras/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
ENV LD_LIBRARY_PATH=$COPPELIASIM_ROOT
ENV QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
RUN sudo ldconfig
RUN sudo apt-get -y install python3-dev python3-opencv libgl1-mesa-dev
RUN sudo python3 -mpip install cffi numpy numpy_indexed
RUN cd extras && git clone --progress https://github.com/stepjam/PyRep.git && cd PyRep && sudo COPPELIASIM_ROOT=/home/robolab/robocomp/components/dsr-graph/extras/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04 python3 -m pip install .
# RUN sudo sed -i 's/-p/-h 172.16.238.5 -p/g' /opt/robocomp/etc/rcnode.conf
# RUN sudo sed -i 's/-p/-h 172.16.238.5 -p/g' /home/robolab/robocomp/components/dsr-graph/components/viriatoPyrep/config




