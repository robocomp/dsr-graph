FROM robocomp/robocomp:focal_dsr_dev

RUN apt-get update -q && \
	export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
        vim tar xz-utils \
        libx11-6 libxcb1 libxau6 libgl1-mesa-dev \
        xvfb dbus-x11 x11-utils libxkbcommon-x11-0 \
        libavcodec-dev libavformat-dev libswscale-dev \
        && \
    apt-get autoclean -y && apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /shared /opt
ARG COPPELIASIM_RELEASE=CoppeliaSim_Edu_V4_2_0_Ubuntu18_04
COPY ./download/$COPPELIASIM_RELEASE.tar.xz /opt/
RUN tar -xf /opt/$COPPELIASIM_RELEASE.tar.xz -C /opt && \
    rm /opt/$COPPELIASIM_RELEASE.tar.xz

ENV COPPELIASIM_ROOT_DIR=/opt/$COPPELIASIM_RELEASE
ENV LD_LIBRARY_PATH=$COPPELIASIM_ROOT_DIR:$LD_LIBRARY_PATH
ENV PATH=$COPPELIASIM_ROOT_DIR:$PATH
COPY --from=robocomp/robocomp:focal_dsr_dev /home/robolab/robocomp/components/dsr-graph/robots_pyrep/ /robots_pyrep/

#RUN echo '#!/bin/bash\ncd $COPPELIASIM_ROOT_DIR\n/usr/bin/xvfb-run --server-args "-ac -screen 0, 1024x1024x24" coppeliaSim "$@"' > /entrypoint && chmod a+x /entrypoint
RUN echo '#!/bin/bash\n(rcnode&) && cd /robots_pyrep/pioneer_pyrep/\nexport SLICE_PATH=./src/ && sh run.sh' > /entrypoint && chmod a+x /entrypoint
RUN apt-get update -q && \
	export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
        python3 python3-zeroc-ice python3-termcolor cmake make git python3-pip python3-setuptools \
        && \
    apt-get autoclean -y && apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN git config --global http.sslVerify false && git clone https://github.com/stepjam/PyRep.git
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT_DIR
ENV QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT_DIR
ENV COPPELIASIM_ROOT=/opt/$COPPELIASIM_RELEASE
RUN export COPPELIASIM_ROOT=/opt/$COPPELIASIM_RELEASE && cd PyRep && pip3 install -r requirements.txt && pip3 install .
RUN pip3 install pytransform3d numpy_indexed opencv-python
RUN pip3 install imutils
EXPOSE 19997
ENTRYPOINT ["/entrypoint"]
