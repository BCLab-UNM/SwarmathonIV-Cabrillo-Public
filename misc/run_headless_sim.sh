#!/bin/bash
#Thanks to G. Matthew Fricke (mfricke@cs.unm.edu) for the initial script

# Function definitions
function userExit() {
	echo "Received SIGINT. Exiting."
	rosnode kill -all
	./cleanup.sh
	exit
} #end userExit

startGazeboServer() {
	local world_file_path=$1
	local random_seed=$2
	rosparam set /use_sim_time true
	setsid rosrun gazebo_ros gzserver $world_file_path --seed $random_seed --verbose &
	echo "Attempted to start Gazebo server with world file: $world_file_path and random seed $random_seed"
} #end startGazeboServer

stopGazebo() {
	pkill gzserver
	echo "Attempted to stop Gazebo server" 
} #end stopGazebo

startGazeboClient() {
	setsid rosrun gazebo_ros gzclient __name:=gzclient &
	echo "Attempted to start Gazebo client"
} #end startGazeboClient

stopGazeboClient() {
	pkill gzclient
	echo "Attempted to stop Gazebo client"
} #end stopGazeboClient

addCollectionZone() {
	setsid rosrun gazebo_ros spawn_model -sdf \
				-file $PWD/simulation/models/collection_disk/model.sdf \
				-model collection_disk -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0
	echo "Attempted to add collection_zone: name=collection_disk, x=0, y=0, z=0, roll=0, pitch=0, yaw=0"
} #end addCollectionZone

addGroundPlane() {
setsid rosrun gazebo_ros spawn_model -sdf -file $PWD/simulation/models/concrete_ground_plane/model.sdf \
			   -model concrete_ground_plane \
			   -x 0 \
			   -y 0 \
			   -z 0 \
			   -R 0 \
			   -P 0 \
			   -Y 0
	echo "Attempted to add concrete ground plane: name=concrete_ground_plane, x=0, y=0, z=0, roll=0, pitch=0, yaw=0"
} #end addGroundPlane

# Stops the ROS nodes associated with rovers
startRoverNodes() {
	local rover_name=$1
	setsid roslaunch $PWD/launch/swarmie.launch name:=$rover_name > logs/$rover_name.log &
	echo "Attempted to start rover ROS nodes"
} #end startRoverNodes

# Stops the ROS nodes associated with rovers
stopRoverNodes() {
	local rover_name=$1
	rosnode kill rover_name_APRILTAG rover_name_BASE2CAM rover_name_DIAGNOSTICS \
				 rover_name_MAP rover_name_BEHAVIOUR rover_name_SBRIDGE \ 
				 rover_name_NAVSAT rover_name_ODOM 
	rosnode cleanup
	echo "Attempted to kill rover ROS nodes: name=$rover_name"
} #end stopRoverNodes

addRover()
{
	local rover_name=$1
	local x=$2
	local y=$3
	local z=$4
	local roll=$5
	local pitch=$6
	local yaw=$7
	
	setsid rosrun gazebo_ros spawn_model -sdf -file $PWD/simulation/models/$rover_name/model.sdf \
		   -model $rover_name \
		   -x $x \
		   -y $y \
		   -z $z \
		   -R $roll \
		   -P $pitch \
		   -Y $yaw
	echo "Attempted to add rover: name=$rover_name, x=$x, y=$y, z=$z, roll=$roll, pitch=$pitch, yaw=$yaw"
}

get_score(){
	score=`rostopic echo -n 1 /collectionZone/score/data | head -n1`
}

#---------------------------------------------------------#
#  Main
#---------------------------------------------------------#

# Exit script if user enters ctl-c or sends interrupt or just exits
trap userExit SIGINT SIGTERM EXIT 

# If not given 4 or 5 arguments then show the usage text
if [ $# -ne 6 -a $# -ne 5 ]
then
	echo "Usage: $0 world_file_path num_rovers(1-8) scoring_output_path experiment_duration_in_minutes random_seed [visualize]"
	echo "Example: $0 simulation/worlds/powerlaw_targets_example.world 6 ~/swarmathon_data/experiment1.txt 30 random_seed visualize"
	echo "Example: $0 simulation/worlds/powerlaw_targets_example.world 6 ~/swarmathon_data/experiment1.txt random_seed 30"   
   exit 1
fi

echo "Running in $PWD" 
previous_gazebo_model_path=${GAZEBO_MODEL_PATH}
previous_gazebo_plugin_path=${GAZEBO_PLUGIN_PATH}
export SWARMATHON_APP_ROOT="$PWD"
export GAZEBO_MODEL_PATH="$PWD/simulation/models"
export GAZEBO_PLUGIN_PATH="$PWD/build/gazebo_plugins"
source "$PWD/devel/setup.bash"
echo Cleaning up ROS and Gazebo Processes
./cleanup.sh
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
roscore &
sleep 2

echo "Experiment started at $(date +%d-%m-%Y" "%H:%M:%S)."

#---------------------------------------------------------#
# Set the interval at which to check whether the experiment duration has elapsed
SLEEP_INTERVAL=.5

# Delay between adding rovers
# The following line set the interval to 2 seconds
MODEL_ADD_INTERVAL=2s

#---------------------------------------------------------#
# The maximum number of rovers a user can request is currently
MAX_ROVERS=8

#---------------------------------------------------------#
# Read the world file path from command line
WORLD_FILE_PATH=$1
echo "World file path: $WORLD_FILE_PATH"

#---------------------------------------------------------#
# Read the random seed to give gazebo
RANDOM_SEED=$5
echo "Random seed: $RANDOM_SEED"

# Start the gazebo simulation
startGazeboServer $WORLD_FILE_PATH $RANDOM_SEED

# Start the gazebo simulation
if [ $# -eq 6 -a "$6" = "visualize" ]
then
	echo "User requested that the Gazebo client be started"
	startGazeboClient
fi

# Read the number of rovers to create from command line
NUM_ROVERS=$2
echo "The user requested $NUM_ROVERS rovers."
if [[ $NUM_ROVERS -gt $MAX_ROVERS ]]; then
	echo "User requested too many rovers. Maximum rovers is $MAX_ROVERS. Exiting."
	exit 2
fi

#---------------------------------------------------------#

addCollectionZone
addGroundPlane

#---------------------------------------------------------#

# Add the rovers to the simulation

#  The distances that determine the X, Y coords of the rovers is determined as follows:
#  The distance to the rover from a corner position is calculated differently
#  than the distance to a cardinal position.
# 
#  The cardinal direction rovers are a straightforward calculation where:
#	  a = the distance to the edge of the collection zone
#		  i.e., 1/2 of the collection zone square side length
#	  b = the 50cm distance required by the rules for placing the rover
#	  c = offset for the simulation for the center of the rover (30cm)
#		  i.e., the rover position is at the center of its body
# 
#  The corner rovers use trigonometry to calculate the distance where each
#  value of d, e, and f, are the legs to an isosceles right triangle. In
#  other words, we are calculating and summing X and Y offsets to position
#  the rover.
#	  d = a
#	  e = xy offset to move the rover 50cm from the corner of the collection zone
#	  f = xy offset to move the rover 30cm to account for its position being
#		  calculated at the center of its body
# 
#                    *  *          d = 0.508m
#                *      *        e = 0.354m
#              *          *    + f = 0.212m
#            *     /*     *    ------------
#            *    / | f *            1.072m
#              * /--| *
#               /* *
#              /  | e
#             /--|
#*************
#*          /|
#*         / |
#*        /  | d                 a = 0.508m
#*       /   |     *********     b = 0.500m
#*      /    |     *       *   + c = 0.300m
#*     *-----|-----*---*   *   ------------
#*        a  *  b  * c     *         1.308m
#*           *     *********
#*           *
#*           *
#*           *
#*************

# Specify rover names
ROVER_NAMES=( "achilles" "aeneas" "ajax" "diomedes" "hector" "paris" "thor" "zeus" )

# Specify rover start coordinates
ROVER_POSITIONS_X=( -1.308 0.000 1.308 0.000 1.072 -1.072 -1.072 1.072 )
ROVER_POSITIONS_Y=( 0.000 -1.308 0.000 1.308 1.072 -1.072 1.072 -1.072 )
 
# In this case, the yaw is the value that turns rover "left" and "right" */
ROVER_YAWS=( 0.000 1.571 -3.142 -1.571 -2.356 0.785 -0.785 2.356 )

echo "Adding rovers to Gazebo and starting their ROS nodes..."

# Add rovers to the simulation and start the associated ROS nodes
for (( i=0;i<$NUM_ROVERS;i++ ));
do
	sleep $MODEL_ADD_INTERVAL
	addRover ${ROVER_NAMES[i]} ${ROVER_POSITIONS_X[i]} ${ROVER_POSITIONS_Y[i]} 0 0 0 ${ROVER_YAWS[i]}
	sleep $MODEL_ADD_INTERVAL
	startRoverNodes ${ROVER_NAMES[i]}
done

echo "Finished adding rovers."

#---------------------------------------------------------#

echo "Setting rovers to autonomous mode..."
# Send the autonomous command to all rovers
for (( i=0;i<$NUM_ROVERS;i++ ));
do
	# Publish the autonomous mode command ("2") to each rover. 
	rostopic pub /${ROVER_NAMES[i]}/mode std_msgs/UInt8 2 & ########********** look at this line
	echo "Publishing 2 on /${ROVER_NAMES[i]}/mode"
	sleep $MODEL_ADD_INTERVAL
done
echo "Finished setting rovers to autonomous mode."

# Read output file path from command line
SCORE_OUTPUT_PATH=$3

mkdir -p $(dirname $SCORE_OUTPUT_PATH)
echo "User specified $SCORE_OUTPUT_PATH as the file to which score information should be appended."

#---------------------------------------------------------#

# Read the current sim time (in seconds) from the ros topic /clock
rosstart=`rostopic echo -n 1 /clock/clock/secs | head -n 1`; start=$(date +%s)
score=0
echo "Time, Score" | tee -a $SCORE_OUTPUT_PATH
# Let the simulation run until the experiment duration is reached arg 4 is the time in mins
until (( $(( $rosnow-$rosstart )) >= $(( $4*60 )) )); do
	get_score #Collect the score over time and write to screen and file
	rosnow=`rostopic echo -n 1 /clock/clock/secs | head -n 1`
	echo "$(( $rosnow-$rosstart )) : $score" | tee -a $SCORE_OUTPUT_PATH
	sleep $SLEEP_INTERVAL
done

echo "Sim Complete, Ending autonomous mode for all rovers."
# Send the manual command to all rovers
for (( i=0;i<$NUM_ROVERS;i++ ));
do
	# Publish the manual mode command ("1") to each rover.
	# Latching mode is the default when using command-line arguments.
	rostopic pub /${ROVER_NAMES[i]}/mode std_msgs/String "1" &
	echo "Publishing 1 on /${ROVER_NAMES[i]}/mode"
done
rosnow=`rostopic echo -n 1 /clock/clock/secs | head -n 1`; now=$(date +%s) #lookinto the --filter 

# The rover program cleans up after itself but if there is a crash this helps to make sure there are no leftovers
echo Cleaning up ROS and Gazebo Processes
rosnode kill --all 
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
./cleanup.sh #TODO: Fix the path for this call
# Restore previous environment
export GAZEBO_MODEL_PATH=$previous_gazebo_model_path
export GAZEBO_PLUGIN_PATH=$previous_gazebo_plugin_path

echo "Sys Time:" $((now - start)) ", Sim Time:"  $((rosnow - rosstart)) | tee -a $SCORE_OUTPUT_PATH
echo "Score:$score" | tee -a $SCORE_OUTPUT_PATH

