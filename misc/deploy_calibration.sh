#! /bin/bash

set -e

if [ -z "$1" ]; then
    echo "usage: $0 <rovername>"
    exit -1
fi

TEMPDIR=$(mktemp -d)

function finish {
  rm -rf "$TEMPDIR"
}

trap finish EXIT

cd $(catkin locate)

# Less than a full launch
echo "Copying calibration files."

cp -R misc/ $TEMPDIR
cp -R arduino/ $TEMPDIR
cat <<EOF > $TEMPDIR/bootstrap.sh
function finish {
  cd ~
  rm -rf "$TEMPDIR"
}

trap finish EXIT

cd $TEMPDIR
pwd
source /opt/ros/kinetic/setup.bash
./misc/rover_onboard_calibration.sh 

EOF
chmod u+x $TEMPDIR/bootstrap.sh

(
    # Copy the build products to the swarmie. 
    cd $TEMPDIR
    tar -cf - $TEMPDIR 2>/dev/null | ssh swarmie@$1 "cd /; tar -xf -"
)

echo "Executing calibration."
# Execute the bootstrap code.
ssh -t swarmie@$1 $TEMPDIR/bootstrap.sh
