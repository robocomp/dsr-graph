ARG branch="development"
FROM robocomp/robocomp:development__dsr__latest
ARG branch
# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp with DSR Components in Ubuntu Focal"
RUN curl -sL https://raw.githubusercontent.com/robocomp/dsr-graph/${branch}/scripts/installation/coppelia_sim_and_pyrep_install.sh | bash -x
RUN cd ~/robocomp/components/
RUN git clone https://github.com/robocomp/dsr-graph/
RUN export ROBOCOMP=~/robocomp/ && cd dsr-graph/robots_pyrep/viriatoPyrep/ && cmake . && make
RUN export ROBOCOMP=~/robocomp/ && cd dsr-graph/robots_pyrep/omniPyrep/ && cmake . && make
RUN export ROBOCOMP=~/robocomp/ && cd dsr-graph/robots_pyrep/pioneer_pyrep/ && cmake . && make

# docker build -t robocomp/robocomp:focal_dsr .
# docker-compose up    

