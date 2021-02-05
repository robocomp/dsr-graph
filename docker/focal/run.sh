set -e
if [ -z "$COPPELIASIM_ROOT" ]; then
    echo "You need to export \$COPPELIASIM_ROOT to the path where you have installed Coppelia SIM"
    exit
fi
xhost +"local:docker@"
docker-compose up &
source /etc/profile.d/robocomp.sh
rccd viriatoPyrep && sh run.sh
