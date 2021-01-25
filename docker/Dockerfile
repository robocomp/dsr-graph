FROM robocomp/robocomp:focal_fcl

# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp with DSR support in Ubuntu Focal"

RUN  echo "robolab:robolab" | chpasswd && adduser robolab sudo
ENV ROBOCOMP /home/robolab/robocomp/

# DSR Dependencies
RUN apt update
RUN apt install -y curl libasio-dev libtinyxml2-dev  libopencv-dev libqglviewer-dev-qt5 libeigen3-dev net-tools vim x11-utils iputils-ping netcat wget python3-dev python3-pybind11
RUN python3 -m pip install pyrep
RUN curl -S https://raw.githubusercontent.com/robocomp/robocomp/development/classes/dsr/dependencies.sh | bash -x
RUN git clone https://github.com/ryanhaining/cppitertools /usr/local/include/cppitertools
COPY Connection.h /usr/include/Ice/Connection.h

#ÇCompilation of DSR libs
USER robolab
# RUN sed -i.bak -e '48d' ~/robocomp/classes/dsr/core/rtps/dsrparticipant.cpp
RUN cd ~/robocomp/classes/dsr && mkdir build && cd build && cmake .. && make -j$(nproc)
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
# RUN cd components/viriatoPyrep && cmake . && make -j$(nproc) && cp etc/config . && sed -i 's/-p/-h 172.16.238.4 -p/g' config
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




