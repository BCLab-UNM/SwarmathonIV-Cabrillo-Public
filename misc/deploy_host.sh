#! /bin/bash

set -e

# Parse arguments
skip_arduino=
skip_build=0
rovername=
for arg in "$@"
do
    case "$arg" in
    --skip-arduino|-s) 
        skip_arduino=--skip-arduino
        ;;
    --skip-build|-f) 
        skip_build=1
        ;;
    *)
	rovername=$arg
        ;;
    esac
done

if [ -z "$rovername" ]; then
    echo "usage: $0 [options] <rovername>"
    echo "  options:"
    echo "    -s | --skip-arduino : Skip loading the Arduino sketch"
    echo "    -f | --skip-build   : Skip building the workspace"
    exit -1
fi

TEMPDIR=$(mktemp -d)

function finish {
  rm -rf "$TEMPDIR"
}

trap finish EXIT

cd $(catkin locate)

# Check that the rover-deploy profile exists. 
if ! catkin profile list | grep -q rover-deploy; then
    echo "Setting up the rover-deploy catkin profile."
    catkin profile add rover-deploy
fi

(
    catkin config --profile rover-deploy -l='deploy/logs'
    catkin config --profile rover-deploy -b='deploy/build'
    catkin config --profile rover-deploy -d='deploy/devel'
    catkin config --profile rover-deploy -i='deploy/install'
    catkin config --profile rover-deploy --install
    catkin config --profile rover-deploy --cmake-args -DCMAKE_BUILD_TYPE=Release
    catkin config --profile rover-deploy --blacklist gazebo_plugins rqt_rover_gui

) > /dev/null

# Build the current workspace
if [ "$skip_build" -eq 0 ]; then
  catkin build --profile rover-deploy --no-status --no-color
fi

# Package the build products, scripts, launch configs and stuff
echo "Copying installation files."

# Launch configurations run from the /proj directory by default.
cp -R deploy/install/ $TEMPDIR
cp -R launch/ $TEMPDIR
cp -R misc/ $TEMPDIR
cp -R arduino/ $TEMPDIR
cp -R src/mobility/resources $TEMPDIR/install/share/mobility/
cat <<EOF > $TEMPDIR/bootstrap.sh
function finish {
  cd ~
  rm -rf "$TEMPDIR"
}

trap finish EXIT

cd $TEMPDIR
pwd
source /opt/ros/kinetic/setup.bash
source ./install/setup.bash
./misc/rover_onboard_node_launch.sh $skip_arduino

EOF
chmod u+x $TEMPDIR/bootstrap.sh

(
    # Copy the build products to the swarmie. 
    cd $TEMPDIR
    tar -cf - $TEMPDIR 2>/dev/null | ssh swarmie@$rovername "cd /; tar -xf -"
)

echo "Executing rover code."
# Execute the bootstrap code.
ssh -t swarmie@$rovername $TEMPDIR/bootstrap.sh
