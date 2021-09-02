set -v
COPPELIA_ROOT_PATH=$(pwd)/extras/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04
sudo apt update
sudo DEBIAN_FRONTEND=noninteractive apt install -y xz-utils python3-dev python3-opencv libgl1-mesa-dev x11-apps
mkdir extras && cd extras && curl https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04.tar.xz --output - | sudo tar -xv -J
export COPPELIASIM_ROOT=$COPPELIA_ROOT_PATH
export LD_LIBRARY_PATH=$COPPELIA_ROOT_PATH
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIA_ROOT_PATH
echo "export COPPELIASIM_ROOT=$COPPELIA_ROOT_PATH" | sudo tee /etc/profile.d/coppelia_path.sh
sudo ldconfig
sudo python3 -mpip install cffi numpy numpy_indexed pytransform3d
git clone --progress https://github.com/stepjam/PyRep.git && cd PyRep && sudo COPPELIASIM_ROOT=$COPPELIA_ROOT_PATH python3 -m pip install .
