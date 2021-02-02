set -v
COPPELIA_ROOT_PATH=$(pwd)/extras/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
sudo apt install xz-utils
mkdir extras && cd extras && wget -c https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz -O - | sudo tar -xv -J
export COPPELIASIM_ROOT=$COPPELIA_ROOT_PATH
export LD_LIBRARY_PATH=$COPPELIA_ROOT_PATH
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIA_ROOT_PATH
sudo echo "\export COPPELIASIM_ROOT=$COPPELIA_ROOT_PATH
export LD_LIBRARY_PATH=$COPPELIASIM_ROOT
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT" >> /etc/profile.d/coppelia_path.sh
sudo ldconfig
sudo apt-get -y install python3-dev python3-opencv libgl1-mesa-dev
sudo python3 -mpip install cffi numpy numpy_indexed
git clone --progress https://github.com/stepjam/PyRep.git && cd PyRep && sudo COPPELIASIM_ROOT=$COPPELIA_ROOT_PATH python3 -m pip install .
