#! /bin/bash

cd ~/robocomp/components/
git clone https://github.com/robocomp/dsr-graph/
cd dsr-graph/components/

sudo apt install libfcl-dev

cd ~/robocomp/build
cmake -D FCL_SUPPORT=1 ..
make
sudo make install

sudo cp ~/robocomp/components/dsr-graph/TriangleFunctor /usr/include/osg

pip3 install opencv-python-headless
