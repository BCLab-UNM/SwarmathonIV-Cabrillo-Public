#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

// Include Controllers
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include "Logger.h"

using namespace std;

/* Drive Parameters. ATTENTION: pay attention to the case of these variable names.
 *
 * These parameters control the driving behavior. They are factored out here for convenience.
 *
 * c_GOAL_THRESHOLD_DISTANCE (meters)
 *    The distance inside of which we're considered to be at our goal.
 *
 * c_TRANSLATE_THRESHOLD_ANGLE (radians)
 *    If the angle between us and the goal is bigger than this the rover goes
 *    into ROTATE to fix the heading.
 *
 * c_ROTATE_THRESHOLD_ANGLE (radians)
 *    The allowable difference between the target angle and theheading. When the
 *    angle is smaller than this the rover goes into TRANSLATE.
 *
 * c_WANDER_RANDOM_ANGLE (radians)
 *    When wandering pick a new heading that's on average this many radians from the current heading.
 *
 * c_WANDER_RANDOM_DISTANCE (meters)
 *    When wandering pick a new goal that's this many meters along the random heading.
 *
 * c_ROTATIONAL_SPEED_MAX
 *    The maximum allowable turning speed.
 *
 * c_ROTAIONAL_SPEED_MIN
 *    The minimum allowable speed while rotating.
 *
 * c_ROTATIONAL_SLOPE
 *    The rotational speed will be:
 *    	speed = max(c_ROTATIONAL_SPEED_MIN, min(c_ROTATIONAL_SPEED_MAX, Angle * c_ROTATIONAL_SLOPE))
 *
 * c_LINEAR SPEED_MAX
 *    The maximum allowable driving speed.
 *
 * c_LINEAR_SPEED_MIN
 *    The minimum allowable driving speed.
 *
 * c_SPEED_SLOPE
 *    The rover linear speed will be:
 *    	speed = max(c_LINEAR_SPEED_MIN, min(c_LINEAR_SPEED_MAX, Distance * c_SPEED_SLOPE))
 *
 *c_BOUNCE_CONST (radians)
 *		The angle the rover turns from the wall when not holding a target
 */
static const double c_GOAL_THRESHOLD_DISTANCE     = 0.1;
static const double c_TRANSLATE_THRESHOLD_ANGLE   = M_PI / 4.0;
static const double c_ROTATE_THRESHOLD_ANGLE      = 0.2;
static const double c_WANDER_RANDOM_ANGLE         = 0.25;
static const double c_WANDER_RANDOM_DISTANCE      = 2.0;
static const double c_ROTATIONAL_SPEED_MAX        = 0.8;
static const double c_ROTATIONAL_SPEED_MIN        = 0.1;
static const double c_ROTATIONAL_SLOPE            = M_PI_4;
static const double c_LINEAR_SPEED_MAX            = 1.0;
static const double c_LINEAR_SPEED_MIN            = 0.2;
static const double c_SPEED_SLOPE                 = 0.5;
static const double c_ROTATE_GIVEUP_SECONDS       = 4.0;
static const double c_TRANSFORM_WAIT_SECONDS      = 0.1;
static const double c_BOUNCE_CONST                = 1.8;
static const int 	c_CIRCLE					  = 105;

// Random number generator
random_numbers::RandomNumberGenerator* rng;

// Create controllers
PickUpController pickUpController;
DropOffController dropOffController;
SearchController searchController;

// Mobility Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees
void mapAverage();  // constantly averages last 100 positions from map

// Numeric Variables for rover positioning
geometry_msgs::Pose2D currentLocation; // odom frame
geometry_msgs::Pose2D currentLocationMap; // GPS frame
geometry_msgs::Pose2D currentLocationAverage; // GPS frame
geometry_msgs::Pose2D currentLocationTotal;
geometry_msgs::Pose2D goalLocation;
geometry_msgs::Pose2D foodLocation; //Where cube was last picked up

//geometry_msgs::Pose2D centerLocation;
geometry_msgs::Pose2D centerLocationMap;
//geometry_msgs::Pose2D centerLocationOdom;

int currentMode = 0;
float mobilityLoopTimeStep = 0.1; // time between the mobility loop calls
float status_publish_interval = 1;
float killSwitchTimeout = 10;
bool targetDetected = false;
bool targetCollected = false;
int backupCount = 0;
int circleCount = 0;
int giveupCount = 0;

//set true when the goal is less than 3 meters
//set false when the goal is more than 3 meters
bool useOdom = false;
float heartbeat_publish_interval = 2;

// Set true when the target block is less than targetDist so we continue
// attempting to pick it up rather than switching to another block in view.
bool lockTarget = false;

// Failsafe state. No legitimate behavior state. If in this state for too long
// return to searching as default behavior.
bool timeOut = false;

// Set to true when the center ultrasound reads less than 0.14m. Usually means
// a picked up cube is in the way.
bool blockBlock = false;

// central collection point has been seen (aka the nest)
bool centerSeen = false;

// Set true when we are insie the center circle and we need to drop the block,
// back out, and reset the boolean cascade.
bool reachedCollectionPoint = false;

// used for calling code once but not in main
bool init = false;

// used to remember place in mapAverage array
int mapCount = 0;

//Used to decide to give up on searching for food
int foodSearchCount = 0;

// How many points to use in calculating the map average position
#define MAP_HISTORY_SIZE 100
unsigned int mapHistorySize = MAP_HISTORY_SIZE;


// An array in which to store map positions
geometry_msgs::Pose2D mapLocation[MAP_HISTORY_SIZE];

bool avoidingObstacle = false;

float searchVelocity = 0.2; // meters/second

std_msgs::String msg;

// state machine states
#define STATE_MACHINE_TRANSFORM	 0
#define STATE_MACHINE_ROTATE	 1
#define STATE_MACHINE_SKID_STEER 2
#define STATE_MACHINE_PICKUP     3
#define STATE_MACHINE_DROPOFF    4
#define STATE_MACHINE_BACKUP	 5
#define STATE_MACHINE_CIRCLE	 6

int stateMachineState = STATE_MACHINE_TRANSFORM;

geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

// Publishers
ros::Publisher status_publisher;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher driveControlPublish;
ros::Publisher heartbeatPublisher;

// Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;

// Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer targetDetectedTimer;
ros::Timer publish_heartbeat_timer;

// records time for delays in sequanced actions, 1 second resolution.
time_t timerStartTime;

// An initial delay to allow the rover to gather enough position data to 
// average its location.
unsigned int startDelayInSeconds = 10;
float timerTimeElapsed = 0;

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetDetectedReset(const ros::TimerEvent& event);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

//function to set goal location and set odom bool
//and overloaded funciton for x, y and theta components
//void setGoalLocation(geometry_msgs::Pose2D location);
//void setGoalLocation(float, float, float);
//Odom based
void setRelativeGoal(double, double);
//GPS based
void setAbsoluteGoal(double, double);
//function to return the currentLocation distinguishing between gps and odom
geometry_msgs::Pose2D& getCurrentLocation();

int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);

    backupCount = 0;
    circleCount = 0;
    giveupCount = 0;

    foodLocation.x = 0;
    foodLocation.y = 0;

    // instantiate random number generator
    rng = new random_numbers::RandomNumberGenerator();
    //set initial random heading

    //select initial search position 50 cm from center (0,0)
    //double distanceR = sqrt(pow(0.5 * cos(goalLocation.theta+M_PI), 2) + pow(0.5 * sin(goalLocation.theta+M_PI), 2));
    setRelativeGoal(.5, rng->uniformReal(0, 2 * M_PI));
    //setRelativeGoal(0.5 * cos(goalLocation.theta+M_PI), 0.5 * sin(goalLocation.theta+M_PI), rng->uniformReal(0, 2 * M_PI));
    //setGoalLocation(0.5 * cos(goalLocation.theta+M_PI), 0.5 * sin(goalLocation.theta+M_PI), rng->uniformReal(0, 2 * M_PI));
    //centerLocation.x = 0;
    //centerLocation.y = 0;
    //centerLocationOdom.x = 0;
    //centerLocationOdom.y = 0;

    for (int i = 0; i < 100; i++) {
        mapLocation[i].x = 0;
        mapLocation[i].y = 0;
        mapLocation[i].theta = 0;
    }

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName
             << "!  Mobility turnDirectionule started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    // Register the SIGINT event handler so the node can shutdown properly
    signal(SIGINT, sigintEventHandler);

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
    mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);

    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
    wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
    driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);
    heartbeatPublisher = mNH.advertise<std_msgs::String>((publishedName + "/mobility/heartbeat"), 1, true);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    targetDetectedTimer = mNH.createTimer(ros::Duration(0), targetDetectedReset, true);

    publish_heartbeat_timer = mNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

    tfListener = new tf::TransformListener();

    Logger::init(publishedName);
    Logger::log("Log Started");
    Logger::log("Rover start delay set to %d seconds", startDelayInSeconds);

    timerStartTime = time(0);

    ros::spin();

    return EXIT_SUCCESS;
}

// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.
void mobilityStateMachine(const ros::TimerEvent&) {

    float rotateOnlyAngleTolerance = 0.4;
    int returnToSearchDelay = 5;

    // Initial computation of key qantities. These may need to be redone if the goal location changes.
    geometry_msgs::Pose2D &currentLocationTemp = getCurrentLocation();
    float distance_to_goal = hypot(goalLocation.x - currentLocationTemp.x, goalLocation.y - currentLocationTemp.y);
    float angle_to_goal = angles::shortest_angular_distance(currentLocationTemp.theta, atan2(goalLocation.y - currentLocationTemp.y, goalLocation.x - currentLocationTemp.x));
    float desired_heading = angles::shortest_angular_distance(currentLocationTemp.theta, goalLocation.theta);

    // Robot is in automode
    if (currentMode == 2 || currentMode == 3) {


        // time since timerStartTime was set to current time
        timerTimeElapsed = time(0) - timerStartTime;

        // init code goes here. (code that runs only once at start of
        // auto mode but wont work in main goes here)
        if (!init) {
        	return;
        }

        // If no collected or detected blocks set fingers
        // to open wide and raised position.
        if (!targetCollected && !targetDetected) {
            // set gripper
            std_msgs::Float32 angle;

            // open fingers
            angle.data = M_PI_2;

            fingerAnglePublish.publish(angle);
            angle.data = 0;

            // raise wrist
            wristAnglePublish.publish(angle);
        }

        // Select rotation or translation based on required adjustment
        switch(stateMachineState) {

        // If no adjustment needed, select new goal
        case STATE_MACHINE_TRANSFORM: {

        	//Should I circle?
            if(circleCount > 0 && !targetCollected){
            	stateMachineState = STATE_MACHINE_CIRCLE;
            	break;
            }

            // If returning with a target
            else if (targetCollected && !avoidingObstacle) {
            	circleCount = 0;
                // calculate the euclidean distance between
                // centerLocation and currentLocation
            	//currentLocationTemp = getCurrentLocation();
            	//################################

                dropOffController.setCenterDist(hypot(centerLocationMap.x - currentLocationMap.x, centerLocationMap.y - currentLocationMap.y));
                dropOffController.setDataLocations(centerLocationMap, currentLocation, currentLocationMap, useOdom, timerTimeElapsed);

                DropOffResult result = dropOffController.getState();

                if (result.timer) {
                    timerStartTime = time(0);
                    reachedCollectionPoint = true;
                }

                std_msgs::Float32 angle;

                if (result.fingerAngle != -1) {
                    angle.data = result.fingerAngle;
                    fingerAnglePublish.publish(angle);
                }

                if (result.wristAngle != -1) {
                    angle.data = result.wristAngle;
                    wristAnglePublish.publish(angle);
                }

                if (result.reset) {
                    timerStartTime = time(0);
                    targetCollected = false;
                    targetDetected = false;
                    lockTarget = false;
                    sendDriveCommand(0.0,0);

                    // move back to transform step
                    stateMachineState = STATE_MACHINE_TRANSFORM;
                    reachedCollectionPoint = false;
                    //centerLocationOdom = getCurrentLocation();

                    dropOffController.reset();
                	Logger::chat("Reset dropoffController.");
                	// XXX: Kiley: This is where the robot freezes after dropping off a target.
                	// I think it should start going back to food here.
                	foodSearchCount = 0;
                	if (!targetDetected && foodLocation.x != 0.0 && foodLocation.y != 0.0){
                		setAbsoluteGoal(foodLocation.x, foodLocation.y);
                	}
                }
                else if(result.goalDriving && timerTimeElapsed >= 30) {
                	timerStartTime = time(0);
                	Logger::chat("LOST");
                	setAbsoluteGoal(centerLocationMap.x, centerLocationMap.y);
                }

                else if (result.goalDriving && timerTimeElapsed >= 1 ) {
                	if(!result.useOdom) {
                		geometry_msgs::Pose2D theGoal = result.centerGoal;
                		setAbsoluteGoal(theGoal.x, theGoal.y);
                		Logger::chat("Going to center");
                	}
                	else {
                		setRelativeGoal(result.centerGoal.x, result.centerGoal.theta);
                		Logger::chat("Searching for center");
                	}
                	stateMachineState = STATE_MACHINE_ROTATE;
                    //timerStartTime = time(0);
                    break;
                }
                else if(result.goalDriving && timerTimeElapsed >= 60) {

                }

                // we are in precision/timed driving
                else {
                	// Set the goal to the current location. This seems to trick
                	// the mobility state machine into staying in the TRANSFORM
                	// state and is vital because this code is driving directly.
                	goalLocation = currentLocation;
                	useOdom = true;
                	sendDriveCommand(result.cmdVel,result.angleError);
                	stateMachineState = STATE_MACHINE_TRANSFORM;
                    break;
                }
            }

            //If angle between current and goal is significant
            //if error in heading is greater than 0.4 radians
            else if (fabs(desired_heading) > rotateOnlyAngleTolerance) {
                stateMachineState = STATE_MACHINE_ROTATE;
            }
            //If goal has not yet been reached drive and maintane heading
            else if (fabs(angle_to_goal) < M_PI_2 && distance_to_goal > c_GOAL_THRESHOLD_DISTANCE) {
                stateMachineState = STATE_MACHINE_SKID_STEER;
            }

            //Otherwise, drop off target and select new random uniform heading
            //If no targets have been detected, assign a new goal
            else if (!targetDetected && timerTimeElapsed > returnToSearchDelay) {

            	// FIXME: From Mike: Take this out of Kiley's branch to separate the food return
            	// behavior from the navigation updates. This code should be put back in when
            	// this feature is tested.
            	//int foodSearchCount = 0;
            	//setAbsoluteGoal(foodLocation.x, foodLocation.y);
            	if (foodLocation.x != 0.0 && foodLocation.y != 0.0){
            		setAbsoluteGoal(foodLocation.x, foodLocation.y);
            		Logger::chat("returning to food (%f %f %f)", foodLocation.x, foodLocation.y, foodSearchCount);
            		if(foodSearchCount >= 2) {
            			stateMachineState = STATE_MACHINE_CIRCLE;
            			circleCount = c_CIRCLE;
            		}

            		foodSearchCount++;
            		if(foodSearchCount > 3) { //search spot for food and then give up
            			foodLocation.x = 0.0;
            			foodLocation.y = 0.0;//this definitely does not work
            		}
            	}
            	else {
            	//
                	if(!targetCollected){
                		circleCount = c_CIRCLE;

            	//geometry_msgs::Pose2D theGoal = searchController.search();
            	//double distanceR = sqrt(pow(theGoal.x, 2) + pow(theGoal.y, 2));
                		setRelativeGoal(2, rng->gaussian(0, 0.25));
                	}
            	//setGoalLocation(searchController.search(currentLocation));
            	}
            	Logger::chat("Picked new random goal.");
            }

            // Re-compute key quantities. The goal location may change inside the transform state.
            currentLocationTemp = getCurrentLocation();
            distance_to_goal = hypot(goalLocation.x - currentLocationTemp.x, goalLocation.y - currentLocationTemp.y);
            angle_to_goal = angles::shortest_angular_distance(currentLocationTemp.theta, atan2(goalLocation.y - currentLocationTemp.y, goalLocation.x - currentLocationTemp.x));
            desired_heading = angles::shortest_angular_distance(currentLocationTemp.theta, goalLocation.theta);

            //Purposefully fall through to next case without breaking
        }

        // Calculate angle between currentLocation.theta and goalLocation.theta
        // Rotate left or right depending on sign of angle
        // Stay in this state until angle is minimized
        case STATE_MACHINE_ROTATE: {
            // If angle > 0.4 radians rotate but dont drive forward.
            if (fabs(desired_heading) > rotateOnlyAngleTolerance) {
                Logger::chat("ROT: Turning to meet desired_heading.");

                // rotate but dont drive  0.05 is to prevent turning in reverse
                sendDriveCommand(0.05, desired_heading);
                break;
            } else {
                // move to differential drive step
                stateMachineState = STATE_MACHINE_SKID_STEER;
                //fall through on purpose.
            }
        }

        // Calculate angle between currentLocation.x/y and goalLocation.x/y
        // Drive forward
        // Stay in this state until angle is at least PI/2
        case STATE_MACHINE_SKID_STEER: {

            // goal not yet reached drive while maintaining proper heading.
            if (fabs(angle_to_goal) < M_PI_2 && distance_to_goal > c_GOAL_THRESHOLD_DISTANCE) {
                Logger::chat("SKS: Driving becaue ange_to_goal is small.");
                // drive and turn simultaniously
                sendDriveCommand(searchVelocity, desired_heading/2);
            }

            // goal is reached but desired heading is still wrong turn only
            else if (fabs(desired_heading) > 0.1) {
                 // rotate but dont drive
                Logger::chat("SKS: Rotating because desired_heading is big.");
                sendDriveCommand(0.0, desired_heading);
            }

            else {
                // stop
                Logger::chat("SKS: Arrived.");
                sendDriveCommand(0.0, 0.0);
                avoidingObstacle = false;

                // move back to transform step
                stateMachineState = STATE_MACHINE_TRANSFORM;
            }

            break;
        }

        case STATE_MACHINE_PICKUP: {
        	Logger::chat("PICKUP");

            PickUpResult result;

            // we see a block and have not picked one up yet
            if (targetDetected && !targetCollected) {
                result = pickUpController.pickUpSelectedTarget(blockBlock);
                sendDriveCommand(result.cmdVel,result.angleError);
                std_msgs::Float32 angle;

                if (result.fingerAngle != -1) {
                    angle.data = result.fingerAngle;
                    fingerAnglePublish.publish(angle);
                }

                if (result.wristAngle != -1) {
                    angle.data = result.wristAngle;

                    // raise wrist
                    wristAnglePublish.publish(angle);
                }

                if (result.giveUp) {
                    targetDetected = false;
                    stateMachineState = STATE_MACHINE_TRANSFORM;
                    sendDriveCommand(0,0);
                    pickUpController.reset();
                }

                if (result.pickedUp) {
                    pickUpController.reset();
                    //currentLocationTemp = getCurrentLocation();
                    // assume target has been picked up by gripper
                    targetCollected = true;
                    result.pickedUp = false;
                    // store coordinates of last pickup for return
                    foodLocation.x = currentLocationMap.x;
                    foodLocation.y = currentLocationMap.y;
                    //foodSearchCount = 0;
        			Logger::chat("setting foodLocation (%f %f)", foodLocation.x, foodLocation.y);


                    stateMachineState = STATE_MACHINE_ROTATE;
                    setAbsoluteGoal(centerLocationMap.x, centerLocationMap.y);

                    // lower wrist to avoid ultrasound sensors
                    std_msgs::Float32 angle;
                    angle.data = 0.8;
                    wristAnglePublish.publish(angle);
                    sendDriveCommand(0.0,0);

                    return;
                }
            } else {
                stateMachineState = STATE_MACHINE_TRANSFORM;
            }

            break;
        }

        case STATE_MACHINE_DROPOFF: {
            Logger::chat("DROPOFF");
            break;
        }

        case STATE_MACHINE_BACKUP: {
        	Logger::chat("BACKING UP");
			if(backupCount > 0){
				backupCount--;
			}
			else {
				stateMachineState = STATE_MACHINE_TRANSFORM;
			}

			// FIXME: Need to replace setVelocity(), how???
			// setVelocity(-0.3, 0.0);
			sendDriveCommand(-0.3, 0.0);

			break;
		}

        case STATE_MACHINE_CIRCLE: {
        	Logger::chat("CIRCLING");
        	circleCount--;
        	if (circleCount <= 0) {
        		stateMachineState = STATE_MACHINE_TRANSFORM;
        	}
        	sendDriveCommand(0.1, 0.3);

        	break;
        }
        default: {
            break;
        }
        } /* end of switch() */
    }
    else { // mode is NOT auto
        // publish current state for the operator to see
    	Logger::chat("WAITING");
    }
}

void sendDriveCommand(double linearVel, double angularError)
{
    velocity.linear.x = linearVel;
    velocity.angular.z = angularError;

    // publish the drive commands
    driveControlPublish.publish(velocity);
}

/*************************
 * ROS CALLBACK HANDLERS *
 *************************/

//Odometry related goal
void setRelativeGoal(double r, double theta) {
	useOdom = true;
	goalLocation.theta = currentLocation.theta + theta;
	goalLocation.x = currentLocation.x + r*cos(goalLocation.theta);
	goalLocation.y = currentLocation.y + r*sin(goalLocation.theta);
	Logger::chat("set relative goal to (%f, %f, %f) current location (%f, %f, %f)", goalLocation.x, goalLocation.y, goalLocation.theta, currentLocation.x, currentLocation.y, currentLocation.theta);
	//goalLocation.x = currentLocation.x + x;
	//goalLocation.y = currentLocation.y + y;
	//goalLocation.theta = currentLocation.theta + theta;
}

//GPS related goal
void setAbsoluteGoal(double x, double y){
	useOdom = false;
	goalLocation.x = x;
	goalLocation.y = y;
	goalLocation.theta = atan2(goalLocation.y - currentLocationMap.y, goalLocation.x - currentLocationMap.x);
	Logger::chat("set absolute goal to (%f, %f, %f) current location (%f, %f, %f)", goalLocation.x, goalLocation.y, goalLocation.theta, currentLocationMap.x, currentLocationMap.y, currentLocationMap.theta);
	//goalLocation.theta = angles::shortest_angular_distance(currentLocationMap.theta, atan2(goalLocation.y - currentLocationMap.y, goalLocation.x - currentLocationMap.x));
	//absolute value?????? should this still be added to current theta
}


void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {

    // If in manual mode do not try to automatically pick up the target
    if (currentMode == 1 || currentMode == 0) return;

    float distance_from_center = hypot(centerLocationMap.x - currentLocation.x, centerLocationMap.y - currentLocation.y);

    if (targetCollected && message->detections.size() > 0 && !avoidingObstacle && stateMachineState == STATE_MACHINE_SKID_STEER) {
    	bool is256 = false;
    	for (int i = 0; i < message->detections.size(); i++) {
    		if(message->detections[i].id == 256) {
    			is256 = true;
    			break;
    		}
    	}
    	if (!is256) {
    		for(int i = 0; i < message->detections.size(); i++) {
    			geometry_msgs::PoseStamped cenPose = message->detections[i].pose;
    			if (cenPose.pose.position.z > .16) {
    				if (cenPose.pose.position.x > 0) {
    					Logger::chat("Turning %f because I see a block in my way!", M_PI_4);
    					setRelativeGoal(.75, M_PI_4);
    				} else {
    					Logger::chat("Turning %f because I see a block in my way!", -M_PI_4);
    					setRelativeGoal(.75, -M_PI_4);
    				}
    				// switch to transform(rotate?) state to trigger collision avoidance
    				stateMachineState = STATE_MACHINE_ROTATE;
    				avoidingObstacle = true;
    				return;
    			}
    		}
    	}

        //setGoalLocation(currentLocation.x, currentLocation.y, currentLocation.theta + c_BOUNCE_CONST);

    }

    // if a target is detected and we are looking for center tags
    if (message->detections.size() > 0 && !reachedCollectionPoint) {
        float cameraOffsetCorrection = 0.020; //meters;

        centerSeen = false;
        int count = 0;
        int countRight = 0;
        int countLeft = 0;

        // this loop is to get the number of center tags
        for (int i = 0; i < message->detections.size(); i++) {
            if (message->detections[i].id == 256) {
                geometry_msgs::PoseStamped cenPose = message->detections[i].pose;

                // checks if tag is on the right or left side of the image
                if (cenPose.pose.position.x + cameraOffsetCorrection > 0) {
                    countRight++;

                } else {
                    countLeft++;
                }

                centerSeen = true;
                count++;
            }
        }

        if (centerSeen && targetCollected) {
        	Logger::chat("Center seen and target collected");
        	goalLocation = currentLocation;
        	useOdom = true;
            stateMachineState = STATE_MACHINE_TRANSFORM;
            circleCount = 0;
        }

        dropOffController.setDataTargets(count,countLeft,countRight);

        // if we see the center and we dont have a target collected
        if (centerSeen && !targetCollected) {

            float centeringTurn = 0.3; //radians
            stateMachineState = STATE_MACHINE_TRANSFORM;
            circleCount = 0;

            // FIXME: Make a better turn away from the center...
            //
            // this code keeps the robot from driving over
            // the center when searching for blocks
            if (!avoidingObstacle) {
            	if (countRight > countLeft) {
            		Logger::chat("Right");
            		// turn away from the center to the left if just driving
            		// around/searching.
            		setRelativeGoal(.75, c_BOUNCE_CONST);
            		avoidingObstacle = true;
            		//setGoalLocation(goalLocation.x, goalLocation.y, goalLocation.theta + centeringTurn);
            		//goalLocation.theta += centeringTurn;
            	} else {
            		Logger::chat("Left");
            		// turn away from the center to the right if just driving
            		// around/searching.
            		setRelativeGoal(.75, -c_BOUNCE_CONST);
            		avoidingObstacle = true;
            		//setGoalLocation(goalLocation.x, goalLocation.y, goalLocation.theta - centeringTurn);
            		//goalLocation.theta -= centeringTurn;
            	}
            }

            // continues an interrupted search
          //  geometry_msgs::Pose2D theGoal = searchController.continueInterruptedSearch(goalLocation);

            //setGoalLocation(searchController.continueInterruptedSearch(getCurrentLocation(), goalLocation));

            targetDetected = false;
            pickUpController.reset();

            return;
        }
    }
    // end found target and looking for center tags

    // found a target april tag and looking for april cubes;
    // with safety timer at greater than 5 seconds.
    PickUpResult result;

    if (message->detections.size() > 0 && !targetCollected && timerTimeElapsed > 5) {
        targetDetected = true;

        // pickup state so target handler can take over driving.
        stateMachineState = STATE_MACHINE_PICKUP;
        result = pickUpController.selectTarget(message);

        std_msgs::Float32 angle;

        if (result.fingerAngle != -1) {
            angle.data = result.fingerAngle;
            fingerAnglePublish.publish(angle);
        }

        if (result.wristAngle != -1) {
            angle.data = result.wristAngle;
            wristAnglePublish.publish(angle);
        }
    }
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
    currentMode = message->data;
    sendDriveCommand(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
    if ((!targetDetected || targetCollected) && (message->data > 0)) {
        // obstacle on right side
        if (message->data == 1) {
            // select new heading 0.2 radians to the left

        	// FIXME:
        	// the line below is redundant:

        	//setGoalLocation(goalLocation.x, goalLocation.y, currentLocation.theta + 0.6);

            // select new heading. If carrying a block, turn c_BOUNCE_CONST (currently 1.8rad) to the LEFT; otherwise 0.6rad to the LEFT
            if (!targetCollected) {
            	setRelativeGoal(0, c_BOUNCE_CONST);
            	//setGoalLocation(currentLocation.x, currentLocation.y, currentLocation.theta + c_BOUNCE_CONST);
            }
            else {
            	setRelativeGoal(0, 0.6);
        		//setGoalLocation(currentLocation.x, currentLocation.y, currentLocation.theta + 0.6);
            }
        }

        // obstacle in front or on left side
        else if (message->data == 2) {

            // select new heading 0.2 radians to the right
        	// FIXME:
        	// the line below is redundant:
        	//setRelativeGoal(goalLocation.x, goalLocation.y, currentLocation.theta + 0.6);

            // select new heading 0.6 radians to the RIGHT.
            if (!targetCollected) {
            	setRelativeGoal(0, -c_BOUNCE_CONST);
            	//setRelativeGoal(currentLocation.x, currentLocation.y, currentLocation.theta - c_BOUNCE_CONST);
            }
            else {
            	setRelativeGoal(0, -0.6);
        		//setRelativeGoal(currentLocation.x, currentLocation.y, currentLocation.theta - 0.6);
            }
        }

        // continues an interrupted search
        //geometry_msgs::Pose2D theGoal = searchController.continueInterruptedSearch(goalLocation);
        //setRelativeGoal(theGoal.x, theGoal.y, theGoal.theta);
       // setGoalLocation(searchController.continueInterruptedSearch(currentLocation, goalLocation));

        // switch to transform state to trigger collision avoidance
        stateMachineState = STATE_MACHINE_ROTATE;
        circleCount = 0;

        avoidingObstacle = true;
    }

    // the front ultrasond is blocked very closely. 0.14m currently
    if (message->data == 4) {
        blockBlock = true;
    } else {
        blockBlock = false;
    }
}

geometry_msgs::Pose2D& getCurrentLocation() {
	if(useOdom)return currentLocation;
	return currentLocationMap;
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {

    //Get (x,y) location directly from pose

    currentLocation.x = message->pose.pose.position.x;
    currentLocation.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.theta = yaw;
}

void mapHandler(const nav_msgs::Odometry::ConstPtr& message) {

	//Get (x,y) location directly from pose
    currentLocationMap.x = message->pose.pose.position.x;
    currentLocationMap.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocationMap.theta = yaw;

    // calls the averaging function, also responsible for
    // transform from Map frame to odom frame.
    mapAverage();
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {
    if (currentMode == 0 || currentMode == 1) {
        sendDriveCommand(abs(message->axes[4]) >= 0.1 ? message->axes[4] : 0, abs(message->axes[3]) >= 0.1 ? message->axes[3] : 0);
    }
}


void publishStatusTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}


void targetDetectedReset(const ros::TimerEvent& event) {
    targetDetected = false;

    std_msgs::Float32 angle;
    angle.data = 0;

    // close fingers
    fingerAnglePublish.publish(angle);

    // raise wrist
    wristAnglePublish.publish(angle);
}

void sigintEventHandler(int sig) {
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

/*
 * FIXME: This function is inefficient. It should use the filter technique
 * for making a running average so that it doesn't have to iterate through
 * this history list every call.
 *
 * Also: IMPORTANT the centerLocation variable is in the rover's odometry
 * frame. That's makes odometry navigation to the center possible but
 * we're switching to map navigation. The centerLocation variable should
 * be changed to be in the Map frame.
 *
 * One more thing: This function is called from mobilityStateMachine() which
 * is wasteful. The only time we need to put a new sample in the averager is
 * when there's a new GPS sample given to mapHandler(). This function
 * should be called from mapHandler()
 *
 */
void mapAverage() {
	//store previous location at mapLocation[mapCount]
	geometry_msgs::Pose2D subMapLocation = mapLocation[mapCount];
    // store currentLocation in the averaging array
    mapLocation[mapCount] = currentLocationMap;
    mapCount++;

    if (mapCount >= mapHistorySize) {
        mapCount = 0;
    }

    //do running count of the average
    currentLocationTotal.x += (mapLocation[mapCount].x-subMapLocation.x);
    currentLocationTotal.y += (mapLocation[mapCount].y-subMapLocation.y);
    currentLocationTotal.theta += (mapLocation[mapCount].theta-subMapLocation.theta);
    currentLocationAverage.x = currentLocationTotal.x/mapHistorySize;
    currentLocationAverage.y = currentLocationTotal.y/mapHistorySize;
    currentLocationAverage.theta = currentLocationTotal.theta/mapHistorySize;
    if (mapCount == 99) {
    	Logger::chat("center location: (%f, %f, %f) ", currentLocationAverage.x, currentLocationAverage.y, currentLocationAverage.theta);
    	centerLocationMap.x = currentLocationAverage.x + 1.1 * cos(currentLocationAverage.theta);
    	centerLocationMap.y = currentLocationAverage.y + 1.1 * sin(currentLocationAverage.theta);
    	centerLocationMap.theta = currentLocationAverage.theta;

    	// initialization has run
    	init = true;
    	mapHistorySize = 5;
    	mapCount = 0;
    }
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
}
