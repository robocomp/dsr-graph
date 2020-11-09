FROM robocomp/robocomp:focal_fcl

# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp with DSR support in Ubuntu Focal"

# DSR Dependencies
RUN apt install -y curl libasio-dev libtinyxml2-dev  libopencv-dev libqglviewer-dev-qt5 libeigen3-dev net-tools
RUN curl -S https://raw.githubusercontent.com/robocomp/robocomp/development/classes/dsr/dependencies.sh | bash -x
RUN git clone https://github.com/ryanhaining/cppitertools /usr/local/include/cppitertools
COPY Connection.h /usr/include/Ice/Connection.h

#
USER robolab
WORKDIR /home/robolab/robocomp/components/
RUN git clone --progress https://github.com/robocomp/dsr-graph.git 
WORKDIR /home/robolab/robocomp/components/dsr-graph/components/
ENV ROBOCOMP /home/robolab/robocomp/
RUN mkdir build && cd build && cmake .. && make
USER root
RUN cd ~/robocomp/classes/dsr && mkdir build && cd build && cmake .. && make && make install
RUN apt -y install vim x11-utils iputils-ping netcat
RUN  echo "robolab:robolab" | chpasswd && adduser robolab sudo
USER robolab
