FROM robocomp/robocomp:focal_fcl

# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp with DSR support in Ubuntu Focal"

ARG DEBIAN_FRONTEND=noninteractive

RUN  echo "robolab:robolab" | chpasswd && adduser robolab sudo
ENV ROBOCOMP /home/robolab/robocomp/

### DSR package Dependencies
RUN apt update
RUN apt install -y curl libasio-dev libtinyxml2-dev  libopencv-dev libqglviewer-dev-qt5 libeigen3-dev net-tools vim x11-utils iputils-ping netcat wget python3-dev python3-pybind11 gnupg2
RUN python3 -m pip install pyrep
### Third party dependencies
RUN curl -S https://raw.githubusercontent.com/robocomp/robocomp/development/classes/dsr/dependencies.sh | bash -x
RUN git clone https://github.com/ryanhaining/cppitertools /usr/local/include/cppitertools

# ICE fix
COPY Connection.h /usr/include/Ice/Connection.h

# Last version of g++ gcc and cmaake
RUN apt install -y cmake gcc-10 g++-10
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
RUN mkdir build && cd build && cmake .. && make -j$(nproc)
WORKDIR /home/robolab/robocomp/components/dsr-graph/




