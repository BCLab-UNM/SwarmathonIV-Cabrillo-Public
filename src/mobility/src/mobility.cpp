#include <ros/ros.h>

//ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//ROS messages
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

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>


using namespace std;

/* Drive Parameters.
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
static const double C_TRANSLATE_THRESHOLD_ANGLE   = M_PI / 4.0;
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

//Random number generator
random_numbers::RandomNumberGenerator* rng;

// Create controllers
PickUpController pickUpController;
DropOffController dropOffController;
SearchController searchController;

//Mobility Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees
void mapAverage();  //constantly averages last 100 positions from map


//Numeric Variables
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D currentLocationMap;
geometry_msgs::Pose2D currentLocationAverage;
geometry_msgs::Pose2D goalLocation;
geometry_msgs::Pose2D centerLocation;
geometry_msgs::Pose2D centerLocationMap;
geometry_msgs::Pose2D centerLocationOdom;
geometry_msgs::Pose2D mapLocation[500];

int currentMode = 0;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 1;
float killSwitchTimeout = 10;
bool targetDetected = false;
bool targetCollected = false;
int backupCount = 0;

//set true when the target block is less than targetDist so we continue attempting to pick it up rather than
//switching to another block that is in view
bool lockTarget = false;

// Failsafe state. No legitimate behavior state. If in this state for too long return to searching as default behavior.
bool timeOut = false;

//set to true when the center ultrasound reads less than 0.14m. Usually means a picked up cube is in the way
bool blockBlock = false;

//central collection point has been seen (aka the nest)
bool centerSeen = false;

//set true when we are insie the center circle and we need to drop the block, back out, and reset the boolean cascade.
bool reachedCollectionPoint = false;

//used for calling code once but not in main
bool init = false;

//used to remember place in mapAverage array
int mapCount = 0;

bool avoidingObstacle = false;

float searchVelocity = 0.2;



std_msgs::String msg;

// state machine states
#define STATE_MACHINE_TRANSFORM	0
#define STATE_MACHINE_ROTATE	1
#define STATE_MACHINE_BACKUP	2
#define STATE_MACHINE_DIFFERENTIAL_DRIVE	3
#define STATE_MACHINE_PICKUP    4
#define STATE_MACHINE_DROPOFF   5

int stateMachineState = STATE_MACHINE_TRANSFORM;

geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

//Publishers
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher infoLogPublisher;
ros::Publisher driveControlPublish;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;


//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer targetDetectedTimer;
time_t timerStartTime; //records time for delays in sequanced actions, 1 second resolution.
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


int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);

    rng = new random_numbers::RandomNumberGenerator(); //instantiate random number generator
    goalLocation.theta = rng->uniformReal(0, 2 * M_PI); //set initial random heading

    //select initial search position 50 cm from center (0,0)
    goalLocation.x = 0.5 * cos(goalLocation.theta+M_PI);
    goalLocation.y = 0.5 * sin(goalLocation.theta+M_PI);

    centerLocation.x = 0;
    centerLocation.y = 0;
    centerLocationOdom.x = 0;
    centerLocationOdom.y = 0;

    for (int i = 0; i < 100; i++)
    {
        mapLocation[i].x = 0;
        mapLocation[i].y = 0;
        mapLocation[i].theta = 0;
    }

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility turnDirectionule started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
    mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);

    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
    wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
    infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
    driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    targetDetectedTimer = mNH.createTimer(ros::Duration(0), targetDetectedReset, true);

    tfListener = new tf::TransformListener();
    std_msgs::String msg;
    msg.data = "Log Started";
    infoLogPublisher.publish(msg);

    timerStartTime = time(0);

    ros::spin();


    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent&) {

    std_msgs::String stateMachineMsg;
    float rotateOnlyAngleTolerance = 0.4;
    int returnToSearchDelay = 5;

    mapAverage(); //calls the averaging function, also responsible for transform from Map frame to odom frame.

   stringstream ss;
   ss << "map center " << centerLocationMap.x << " : " << centerLocationMap.y << " centerLocation " << centerLocation.x << " : " << centerLocation.y << " currentLlocation " << currentLocation.x << " : " << currentLocation.y << " currentLocationAverage " << currentLocationAverage.x << " : " << currentLocationAverage.y << " curMap " << currentLocationMap.x << " : " << currentLocationMap.y;
   msg.data = ss.str();
   infoLogPublisher.publish(msg);


    if (currentMode == 2 || currentMode == 3) { //Robot is in automode
        stringstream ss;
        ss << "map center " << centerLocationMap.x << " : " << centerLocationMap.y << " centerLocation " << centerLocation.x << " : " << centerLocation.y << " currentLlocation " << currentLocation.x << " : " << currentLocation.y << " currentLocationAverage " << currentLocationAverage.x << " : " << currentLocationAverage.y << "curMap " << currentLocationMap.x << " : " << currentLocationMap.y;
        msg.data = ss.str();
        infoLogPublisher.publish(msg);

        timerTimeElapsed = time(0) - timerStartTime; //time since timerStartTime was set to current time

        if (!init) //initiliation code goes here. (code that runs only once at start of auto mode but wont work in main goes here)
        {
            if (timerTimeElapsed > 60) {
                centerLocationMap.x = currentLocationAverage.x; // set the location of the center circle location in the map frame
                centerLocationMap.y = currentLocationAverage.y; // based upon our current average location on the map.
                centerLocationMap.theta = currentLocationAverage.theta;
                init = true; //initiliation has run

            }
            else
            {
                return;
            }

        }

        if (!targetCollected && !targetDetected) //if no collected or detected blocks set fingers to open wide and raised position.
        {
            //set gripper
            std_msgs::Float32 angle;
            //open fingers
            angle.data = M_PI_2;
            fingerAnglePublish.publish(angle);
            angle.data = 0;
            wristAnglePublish.publish(angle); //raise wrist
        }


        switch(stateMachineState) {

        //Select rotation or translation based on required adjustment

        //If no adjustment needed, select new goal
        case STATE_MACHINE_TRANSFORM: {
            stateMachineMsg.data = "TRANSFORMING";

            //If returning with a target
            if (targetCollected && !avoidingObstacle) { // && !centerSeen && !reachedCollectionPoint) {


                //calculate the euclidean distance between centerLocation and currentLocation
                dropOffController.setCenterDist(hypot(centerLocation.x - currentLocation.x, centerLocation.y - currentLocation.y));
                dropOffController.setDataLocations(centerLocation, currentLocation, timerTimeElapsed);

                DropOffResult result = dropOffController.getState();

                if (result.timer) {
                    timerStartTime = time(0);
                    reachedCollectionPoint = true;
                }
                std_msgs::Float32 angle;
                if (result.fingerAngle != -1)
                {
                    angle.data = result.fingerAngle;
                    fingerAnglePublish.publish(angle);
                }
                if (result.wristAngle != -1)
                {
                    angle.data = result.wristAngle;
                    wristAnglePublish.publish(angle);
                }

                if (result.reset) {

                    timerStartTime = time(0);
                    targetCollected = false;
                    targetDetected = false;
                    lockTarget = false;
                    sendDriveCommand(0.0,0);
                    stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step
                    reachedCollectionPoint = false;;
                    centerLocationOdom = currentLocation;


                    dropOffController.reset();

                }
                else if (result.goalDriving && timerTimeElapsed >= 5 )
                {

                    goalLocation = result.centerGoal;
                    stateMachineState = STATE_MACHINE_ROTATE;
                    timerStartTime = time(0);
                }
                else //we are in precision/timed driving
                {
                    goalLocation = currentLocation;
                    sendDriveCommand(result.cmdVel,result.angleError);
                    stateMachineState = STATE_MACHINE_TRANSFORM;

                    break;
                }

            }
            //If angle between current and goal is significant
            //if error in heading is greater than 0.4 radians
            else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > rotateOnlyAngleTolerance)
            {
                stateMachineState = STATE_MACHINE_ROTATE; //rotate
            }
            //If goal has not yet been reached drive and maintane heading
            else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) //pi/2
            {
                stateMachineState = STATE_MACHINE_DIFFERENTIAL_DRIVE; //differential drive
            }

            //Otherwise, drop off target and select new random uniform heading
            //If no targets have been detected, assign a new goal
            else if (!targetDetected && timerTimeElapsed > returnToSearchDelay) {
                goalLocation = searchController.search(currentLocation);
            }

            //Purposefully fall through to next case without breaking
        }

            //Calculate angle between currentLocation.theta and goalLocation.theta
            //Rotate left or right depending on sign of angle
            //Stay in this state until angle is minimized
        case STATE_MACHINE_ROTATE: {
            stateMachineMsg.data = "ROTATING";
            //calculate the diffrence between current and desired heading in radians.
            float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);
            //if angle is greater than 0.4 radians rotate but dont drive forward.
            if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > rotateOnlyAngleTolerance)
            {
                sendDriveCommand(0.05, errorYaw); //rotate but dont drive  0.05 is to prevent turning in reverse
                break;
            }
            else {
                stateMachineState = STATE_MACHINE_DIFFERENTIAL_DRIVE; //move to differential drive step
                //fall through on purpose.
            }

        }

            //Calculate angle between currentLocation.x/y and goalLocation.x/y
            //Drive forward
            //Stay in this state until angle is at least PI/2
        case STATE_MACHINE_DIFFERENTIAL_DRIVE: {
            stateMachineMsg.data = "DIFFERENTIAL_DRIVE";
            //calculate the distance between current and desired heading in radians
            float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);
            //goal not yet reached drive while maintaining proper heading.
            if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
                sendDriveCommand(searchVelocity, errorYaw/2); //drive and turn simultaniously
            }
            //goal is reached but desired heading is still wrong turn only
            else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
                sendDriveCommand(0.0, errorYaw); //rotate but dont drive
            }
            else {
                sendDriveCommand(0.0, 0.0); //stop
                avoidingObstacle = false;

                stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step

            }
            break;
        }
        case STATE_MACHINE_PICKUP: {
            stateMachineMsg.data = "PICKUP";

            PickUpResult result;

            if (targetDetected && !targetCollected) //we see a block and have not picked one up yet
            {

                result = pickUpController.pickUpSelectedTarget(blockBlock);

                sendDriveCommand(result.cmdVel,result.angleError);

                std_msgs::Float32 angle;
                if (result.fingerAngle != -1)
                {
                    angle.data = result.fingerAngle;
                    fingerAnglePublish.publish(angle);
                }
                if (result.wristAngle != -1)
                {
                    angle.data = result.wristAngle;
                    wristAnglePublish.publish(angle); //raise wrist
                }

                if (result.giveUp)
                {
                    targetDetected = false;
                    stateMachineState = STATE_MACHINE_TRANSFORM;
                    sendDriveCommand(0,0);
                    pickUpController.reset();
                }

                if (result.pickedUp)
                {
                    pickUpController.reset(); //reset
                    //assume target has been picked up by gripper
                    targetCollected = true;
                    result.pickedUp = false;
                    stateMachineState = STATE_MACHINE_ROTATE;

                    goalLocation.theta = atan2(centerLocationOdom.y - currentLocation.y, centerLocationOdom.x - currentLocation.x);

                    //set center as goal position
                    goalLocation.x = centerLocationOdom.x = 0;
                    goalLocation.y = centerLocationOdom.y;

                    //lower wrist to avoid ultrasound sensors
                    std_msgs::Float32 angle;
                    angle.data = 0.8;
                    wristAnglePublish.publish(angle);
                    sendDriveCommand(0.0,0);

                    return;
                }
            }
            else
            {
                stateMachineState = STATE_MACHINE_TRANSFORM;
            }
            break;
        }
        case STATE_MACHINE_DROPOFF: {
            stateMachineMsg.data = "DROPOFF";

            break;
        }
		case STATE_MACHINE_BACKUP: {
			if(backupCount > 0){
				backupCount--;
			}
			else {
				stateMachineState = STATE_MACHINE_TRANSFORM;
			}

			// FIXME: Need to replace setVelocity(), how???
			// setVelocity(-0.3, 0.0);

			break;
		}
        default: {
            break;
        }
        }
    }
    else { // mode is NOT auto

        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
    }

    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
        stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }
}

void sendDriveCommand(double linearVel, double angularError)
{
    velocity.linear.x = linearVel,
            velocity.angular.z = angularError;
    driveControlPublish.publish(velocity); //publish the drive commands
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {

    // If in manual mode do not try to automatically pick up the target
    if (currentMode == 1 || currentMode == 0) return;

    //if a target is detected and we are looking for center tags
    if (message->detections.size() > 0 && !reachedCollectionPoint)
    {
        float cameraOffsetCorrection = 0.020; //meters;
        bool right = false;
        bool left = false;

        centerSeen = false;
        double count = 0;

        for (int i = 0; i < message->detections.size(); i++) //this loop is to get the number of center tags
        {
            if (message->detections[i].id == 256)
            {
                geometry_msgs::PoseStamped cenPose = message->detections[i].pose;
                if (cenPose.pose.position.x + cameraOffsetCorrection > 0) //checks if tag is on the right or left side of the image
                {
                    right = true;
                }
                else
                {
                    left = true;
                }
                centerSeen = true;
                count++;
            }
        }

        if (centerSeen && targetCollected)
        {
            stateMachineState = STATE_MACHINE_TRANSFORM;
            goalLocation = currentLocation;
        }

        dropOffController.setDataTargets(count,left,right);

        //if we see the center and we dont have a target collected
        if (centerSeen && !targetCollected)
        {
            float centeringTurn = 0.15; //radians
            stateMachineState = STATE_MACHINE_TRANSFORM;

            //this code keeps the robot from driving over the center when searching for blocks
            if (right)
            {
                goalLocation.theta += centeringTurn; //turn away from the center to the left if just driving around/searching.
            }
            else
            {
                goalLocation.theta -= centeringTurn; //turn away from the center to the right if just driving around/searching.
            }

            // continues an interrupted search
            goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation);

            targetDetected = false;
            pickUpController.reset();
            return;
        }
    }
    //end found target and looking for center tags


    //found a target april tag and looking for april cubes; with safety timer at greater than 5 seconds.
    PickUpResult result;

    if (message->detections.size() > 0 && !targetCollected && timerTimeElapsed > 5)
    {

        targetDetected = true;
        stateMachineState = STATE_MACHINE_PICKUP; //pickup state so target handler can take over driving.
        result = pickUpController.selectTarget(message);

        std_msgs::Float32 angle;
        if (result.fingerAngle != -1)
        {
            angle.data = result.fingerAngle;
            fingerAnglePublish.publish(angle);
        }
        if (result.wristAngle != -1)
        {
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
        //obstacle on right side
        if (message->data == 1) {
            //select new heading 0.2 radians to the left
            goalLocation.theta = currentLocation.theta + 0.6;
        }

        //obstacle in front or on left side
        else if (message->data == 2) {
            //select new heading 0.2 radians to the right
            goalLocation.theta = currentLocation.theta + 0.6;
        }

        // continues an interrupted search
        goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation);

        //switch to transform state to trigger collision avoidance
        stateMachineState = STATE_MACHINE_ROTATE;

        avoidingObstacle = true;
    }


    if (message->data == 4) //the front ultrasond is blocked very closely. 0.14m currently
    {
        blockBlock = true;
    }
    else
    {
        blockBlock = false;
    }
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
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {
    if (currentMode == 0 || currentMode == 1) {
        sendDriveCommand(abs(message->axes[4]) >= 0.1 ? message->axes[4] : 0, abs(message->axes[3]) >= 0.1 ? message->axes[3] : 0);
    }
}


void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}


void targetDetectedReset(const ros::TimerEvent& event) {
    targetDetected = false;

    std_msgs::Float32 angle;
    angle.data = 0;
    fingerAnglePublish.publish(angle); //close fingers
    wristAnglePublish.publish(angle); //raise wrist
}

void sigintEventHandler(int sig)
{
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void mapAverage()
{
    mapLocation[mapCount] = currentLocationMap; //store currentLocation in the averaging array
    mapCount++;

    if (mapCount >= 500) {mapCount = 0;}

    double x = 0;
    double y = 0;
    double theta = 0;
    for (int i = 0; i < 500; i++) //add up all the positions in the array
    {
        x += mapLocation[i].x;
        y += mapLocation[i].y;
        theta += mapLocation[i].theta;
    }
    x = x/500; //find the average
    y = y/500;
    theta = theta/100;//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    currentLocationAverage.x = x;
    currentLocationAverage.y = y;
    currentLocationAverage.theta = theta;


    if (init) //only run below code if a centerLocation has been set by initilization
    {
        geometry_msgs::PoseStamped mapPose; //map frame
        mapPose.header.stamp = ros::Time::now(); //setup msg to represent the center location in map frame
        mapPose.header.frame_id = publishedName + "/map";
        mapPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, centerLocationMap.theta);
        //mapPose.pose.orientation.x = 0;
        //mapPose.pose.orientation.y = 0;
        //mapPose.pose.orientation.z = 0;
        //mapPose.pose.orientation.w = 1;
        mapPose.pose.position.x = centerLocationMap.x;
        mapPose.pose.position.y = centerLocationMap.y;
        geometry_msgs::PoseStamped odomPose;
        string x = "";

        try { //attempt to get the transform of the center point in map frame to odom frame.
            tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
            tfListener->transformPose(publishedName + "/odom", mapPose, odomPose);
        }

        catch(tf::TransformException& ex) {
            ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
            x = "Exception thrown " + (string)ex.what();
            std_msgs::String msg;
            stringstream ss;
            ss << "Exception in mapAverage() " + (string)ex.what();
            msg.data = ss.str();
            infoLogPublisher.publish(msg);
        }

    bool useCustomTransform = true;
    if (useCustomTransform)
    {
        geometry_msgs::Pose2D offset;
        offset.x = currentLocation.x - currentLocationMap.x;
        offset.y = currentLocation.y - currentLocationMap.y;

        centerLocation.x = offset.x - centerLocationMap.x;
        centerLocation.y = offset.y - centerLocationMap.y;

    }
    else
    {
        centerLocation.x = odomPose.pose.position.x; //set centerLocation in odom frame
        centerLocation.y = odomPose.pose.position.y;
    }

    }
}




//below is some saved example code
//************************************************************************************************************************************************

//This is code for map link to odom link
/*
    geometry_msgs::PoseStamped mapOrigin;
    mapOrigin.header.stamp = ros::Time::now();
    mapOrigin.header.frame_id = publishedName + "/map";
    mapOrigin.pose.orientation.w = 1;
    geometry_msgs::PoseStamped odomPose;
    string x = "";

        try {
            tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
            tfListener->transformPose(publishedName + "/odom", mapOrigin, odomPose);
        }

        catch(tf::TransformException& ex) {
            ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
            x = "Exception thrown " + (string)ex.what();
        }
    x = odomPose.pose.position.x;

*/


//This is code for camera link to odom link
/*
        geometry_msgs::PoseStamped mapOrigin;
        geometry_msgs::PoseStamped odomPose;
        try {
            tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time(0), ros::Duration(1.0));
            tfListener->transformPose(publishedName + "/odom", mapOrigin, odomPose);
        }
        catch(tf::TransformException& ex) {
            ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
        }
        //x coord = odomPose.pose.position.x;
*/

/*
   stringstream ss;
   ss << "map center " << centerLocationMap.x << " : " << centerLocationMap.y << " centerLocation " << centerLocation.x << " : " << centerLocation.y << " currentLlocation " << currentLocation.x << " : " << currentLocation.y << " currentLocationAverage " << currentLocationAverage.x << " : " << currentLocationAverage.y << " curMap " << currentLocationMap.x << " : " << currentLocationMap.y;
   msg.data = ss.str();
   infoLogPublisher.publish(msg);
   */

