#include <math.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

//ROS libraries
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>

//ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt8.h>

//Package include
#include <usbSerial.h>

#include "pid.h"
#include <bridge/DriveConfig.h>

using namespace std;

//aBridge functions
void driveCommandHandler(const geometry_msgs::Twist::ConstPtr& message);
void fingerAngleHandler(const std_msgs::Float32::ConstPtr& angle);
void wristAngleHandler(const std_msgs::Float32::ConstPtr& angle);
void serialActivityTimer(const ros::TimerEvent& e);
void publishRosTopics();
void parseData(string data);
void initialconfig();

//Globals
geometry_msgs::QuaternionStamped fingerAngle;
geometry_msgs::QuaternionStamped wristAngle;
sensor_msgs::Imu imu;
nav_msgs::Odometry odom;
sensor_msgs::Range sonarLeft;
sensor_msgs::Range sonarCenter;
sensor_msgs::Range sonarRight;
USBSerial usb;
const int baud = 115200;
char dataCmd[] = "d\n";
char moveCmd[16];
char host[128];
const float deltaTime = 0.1; //abridge's update interval
int currentMode = 0;
string publishedName;
geometry_msgs::Twist speedCommand;

float heartbeat_publish_interval = 2;

const double wheelBase = 0.278; //distance between left and right wheels (in M)
const double wheelDiameter = 0.122; //diameter of wheel (in M)
const int cpr = 8400; //"cycles per revolution" -- number of encoder increments per one wheel revolution

double leftTicks = 0;
double rightTicks = 0;
double odomTS = 0;

// Immobilize robot until the first PID configuration.
PID left_pid(0, 0, 0, 0, 255, -255, 0, -1);
PID right_pid(0, 0, 0, 0, 255, -255, 0, -1);

//Publishers
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher imuPublish;
ros::Publisher odomPublish;
ros::Publisher sonarLeftPublish;
ros::Publisher sonarCenterPublish;
ros::Publisher sonarRightPublish;
ros::Publisher infoLogPublisher;
ros::Publisher heartbeatPublisher;

ros::Publisher debugPIDPublisher;
geometry_msgs::Twist dbT;

//Subscribers
ros::Subscriber driveControlSubscriber;
ros::Subscriber fingerAngleSubscriber;
ros::Subscriber wristAngleSubscriber;
ros::Subscriber modeSubscriber;

//Timers
ros::Timer publishTimer;
ros::Timer publish_heartbeat_timer;

//Callback handlers
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void reconfigure(bridge::DriveConfig &cfg, uint32_t level);

int main(int argc, char **argv) {
    
    gethostname(host, sizeof (host));
    string hostname(host);
    ros::init(argc, argv, (hostname + "_ABRIDGE"));
    
    ros::NodeHandle param("~");
    string devicePath;
    param.param("device", devicePath, string("/dev/ttyUSB0"));
    usb.openUSBPort(devicePath, baud);
    void modeHandler(const std_msgs::UInt8::ConstPtr& message);
    
    sleep(5);
    
    ros::NodeHandle aNH;
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  ABridge module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }
    
    fingerAnglePublish = aNH.advertise<geometry_msgs::QuaternionStamped>((publishedName + "/fingerAngle/prev_cmd"), 10);
    wristAnglePublish = aNH.advertise<geometry_msgs::QuaternionStamped>((publishedName + "/wristAngle/prev_cmd"), 10);
    imuPublish = aNH.advertise<sensor_msgs::Imu>((publishedName + "/imu"), 10);
    odomPublish = aNH.advertise<nav_msgs::Odometry>((publishedName + "/odom"), 10);
    sonarLeftPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/sonarLeft"), 10);
    sonarCenterPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/sonarCenter"), 10);
    sonarRightPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/sonarRight"), 10);
    infoLogPublisher = aNH.advertise<std_msgs::String>("/infoLog", 1, true);
    heartbeatPublisher = aNH.advertise<std_msgs::String>((publishedName + "/abridge/heartbeat"), 1, true);
    
    debugPIDPublisher = aNH.advertise<geometry_msgs::Twist>((publishedName + "/abridge/debugPID"), 1, false);

    heartbeatPublisher = aNH.advertise<std_msgs::String>((publishedName + "/abridge/heartbeat"), 1, true);

    driveControlSubscriber = aNH.subscribe((publishedName + "/driveControl"), 10, driveCommandHandler);
    fingerAngleSubscriber = aNH.subscribe((publishedName + "/fingerAngle/cmd"), 1, fingerAngleHandler);
    wristAngleSubscriber = aNH.subscribe((publishedName + "/wristAngle/cmd"), 1, wristAngleHandler);
    modeSubscriber = aNH.subscribe((publishedName + "/mode"), 1, modeHandler);

    
    publishTimer = aNH.createTimer(ros::Duration(deltaTime), serialActivityTimer);
    publish_heartbeat_timer = aNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);
    
    imu.header.frame_id = publishedName+"/base_link";
    
    odom.header.frame_id = publishedName+"/odom";
    odom.child_frame_id = publishedName+"/base_link";

    // configure dynamic reconfiguration
    dynamic_reconfigure::Server<bridge::DriveConfig> config_server;
    dynamic_reconfigure::Server<bridge::DriveConfig>::CallbackType f;
    f = boost::bind(&reconfigure, _1, _2);
    config_server.setCallback(f);

    boost::thread t(initialconfig);
    ros::spin();
    
    return EXIT_SUCCESS;
}

void reconfigure(bridge::DriveConfig &cfg, uint32_t level) {
	double p, i, d, db, st, wu;
	p = cfg.groups.pid.Kp * cfg.groups.pid.scale;
	i = cfg.groups.pid.Ki * cfg.groups.pid.scale;
	d = cfg.groups.pid.Kd * cfg.groups.pid.scale;
	db = cfg.groups.pid.db;
	st = cfg.groups.pid.st;
	wu = cfg.groups.pid.wu;

	left_pid.reconfig(p, i, d, db, st, wu);
	right_pid.reconfig(p, i, d, db, st, wu);

	std_msgs::String msg;
	msg.data = "PID reconfigure is done.";
	infoLogPublisher.publish(msg);
}

void driveCommandHandler(const geometry_msgs::Twist::ConstPtr& message) {
  speedCommand.linear.x = message->linear.x;
  speedCommand.angular.y = message->angular.y;
  speedCommand.angular.z = message->angular.z;
}

// The finger and wrist handlers receive gripper angle commands in floating point
// radians, write them to a string and send that to the arduino
// for processing.
void fingerAngleHandler(const std_msgs::Float32::ConstPtr& angle) {
  char cmd[16]={'\0'};

  // Avoid dealing with negative exponents which confuse the conversion to string by checking if the angle is small
  if (angle->data < 0.01) {
    // 'f' indicates this is a finger command to the arduino
    sprintf(cmd, "f,0\n");
  } else {
    sprintf(cmd, "f,%.4g\n", angle->data);
  }
  usb.sendData(cmd);
  memset(&cmd, '\0', sizeof (cmd));
}

void wristAngleHandler(const std_msgs::Float32::ConstPtr& angle) {
    char cmd[16]={'\0'};

    // Avoid dealing with negative exponents which confuse the conversion to string by checking if the angle is small
  if (angle->data < 0.01) {
    // 'w' indicates this is a wrist command to the arduino
    sprintf(cmd, "w,0\n");
  } else {
    sprintf(cmd, "w,%.4g\n", angle->data);
  }
  usb.sendData(cmd);
  memset(&cmd, '\0', sizeof (cmd));
}

double ticksToMeters(double ticks) {
	return (M_PI * wheelDiameter * ticks) / cpr;
}

double metersToTicks(double meters) {
	return (meters * cpr) / (M_PI * wheelDiameter);
}

double diffToTheta(double right, double left) {
	return (right - left) / wheelBase;
}

double thetaToDiff(double theta) {
	return theta * wheelBase;
}

void serialActivityTimer(const ros::TimerEvent& e) {

	int cmd_mode = round(speedCommand.angular.y);

	if (cmd_mode == DRIVE_MODE_STOP) {
		left_pid.reset();
		right_pid.reset();

		sprintf(moveCmd, "s\n");
		usb.sendData(moveCmd);
		memset(&moveCmd, '\0', sizeof (moveCmd));
	}
	else {
		// Calculate tick-wise velocities.
	        double linear_sp = metersToTicks(speedCommand.linear.x);
		double angular_sp = metersToTicks(thetaToDiff(speedCommand.angular.z));

		int l = round(left_pid.step(linear_sp - angular_sp, leftTicks, odomTS));
		int r = round(right_pid.step(linear_sp + angular_sp, rightTicks, odomTS));

		// Debugging: Report PID performance for tuning.
		// Output of the PID is in Linear:
		dbT.linear.x = l;
		dbT.linear.y = r;
		dbT.linear.z = linear_sp;
		
		// Feedback function is in Angular:
		dbT.angular.x = leftTicks;
		dbT.angular.y = rightTicks;
		dbT.angular.z = rightTicks - leftTicks;
		debugPIDPublisher.publish(dbT);

		sprintf(moveCmd, "v,%d,%d\n", l, r); //format data for arduino into c string
		usb.sendData(moveCmd);                      //send movement command to arduino over usb
		memset(&moveCmd, '\0', sizeof (moveCmd));   //clear the movement command string
	}

	usb.sendData(dataCmd);
	parseData(usb.readData());
	publishRosTopics();
}

void publishRosTopics() {
    fingerAnglePublish.publish(fingerAngle);
    wristAnglePublish.publish(wristAngle);
    imuPublish.publish(imu);
    odomPublish.publish(odom);
    sonarLeftPublish.publish(sonarLeft);
    sonarCenterPublish.publish(sonarCenter);
    sonarRightPublish.publish(sonarRight);
}

void parseData(string str) {
    istringstream oss(str);
    string sentence;
    static double lastOdomTS = 0;
    static double odomTheta = 0;

    while (getline(oss, sentence, '\n')) {
		istringstream wss(sentence);
		string word;

		vector<string> dataSet;
		while (getline(wss, word, ',')) {
			dataSet.push_back(word);
		}

		if (dataSet.size() >= 3 && dataSet.at(1) == "1") {

            if (dataSet.at(0) == "GRF") {
                fingerAngle.header.stamp = ros::Time::now();
                fingerAngle.quaternion = tf::createQuaternionMsgFromRollPitchYaw(atof(dataSet.at(2).c_str()), 0.0, 0.0);
            }
			else if (dataSet.at(0) == "GRW") {
				wristAngle.header.stamp = ros::Time::now();
				wristAngle.quaternion = tf::createQuaternionMsgFromRollPitchYaw(atof(dataSet.at(2).c_str()), 0.0, 0.0);
			}
			else if (dataSet.at(0) == "IMU") {
				imu.header.stamp = ros::Time::now();
				imu.linear_acceleration.x = atof(dataSet.at(2).c_str());
				imu.linear_acceleration.y = 0; //atof(dataSet.at(3).c_str());
				imu.linear_acceleration.z = atof(dataSet.at(4).c_str());
				imu.angular_velocity.x = atof(dataSet.at(5).c_str());
				imu.angular_velocity.y = atof(dataSet.at(6).c_str());
				imu.angular_velocity.z = atof(dataSet.at(7).c_str());
				imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(dataSet.at(8).c_str()), atof(dataSet.at(9).c_str()), atof(dataSet.at(10).c_str()));
			}
			else if (dataSet.at(0) == "ODOM") {
				leftTicks = atoi(dataSet.at(2).c_str());
				rightTicks = atoi(dataSet.at(3).c_str());
				odomTS = atof(dataSet.at(4).c_str()) / 1000; // Seconds

				double rightWheelDistance = ticksToMeters(rightTicks);
				double leftWheelDistance = ticksToMeters(leftTicks);

			    //Calculate relative angle that robot has turned
				double dtheta = diffToTheta(rightWheelDistance, leftWheelDistance);

			    //Accumulate angles to calculate absolute heading
			    odomTheta += dtheta;

			    //Decompose linear distance into its component values
			    double meanWheelDistance = (rightWheelDistance + leftWheelDistance) / 2;
			    double x = meanWheelDistance * cos(dtheta);
			    double y = meanWheelDistance * sin(dtheta);

			    // Calculate velocities if possible.
			    double vtheta = 0;
			    double vx = 0;
			    double vy = 0;
			    if (lastOdomTS > 0) {
			    	double deltaT = odomTS - lastOdomTS;
			    	vtheta = dtheta / deltaT;
			    	vx = x / deltaT;
			    	vy = y / deltaT;

				// Normalize ticks to ticks/s
				leftTicks = leftTicks / deltaT;
				rightTicks = rightTicks / deltaT;
			    }
		    	lastOdomTS = odomTS;

				odom.header.stamp = ros::Time::now();
				odom.pose.pose.position.x += x;
				odom.pose.pose.position.y += y;
				odom.pose.pose.position.z = 0;
				odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odomTheta);

				odom.twist.twist.linear.x = vx;
				odom.twist.twist.linear.y = vy;
				odom.twist.twist.angular.z = vtheta;
			}
			else if (dataSet.at(0) == "USL") {
				sonarLeft.header.stamp = ros::Time::now();
				sonarLeft.range = atof(dataSet.at(2).c_str()) / 100.0;
			}
			else if (dataSet.at(0) == "USC") {
				sonarCenter.header.stamp = ros::Time::now();
				sonarCenter.range = atof(dataSet.at(2).c_str()) / 100.0;
			}
			else if (dataSet.at(0) == "USR") {
				sonarRight.header.stamp = ros::Time::now();
				sonarRight.range = atof(dataSet.at(2).c_str()) / 100.0;
			}

		}
	}
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
	currentMode = message->data;
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
}

void initialconfig() {
    // Set PID parameters from the launch configuration
    bridge::DriveConfig initial_config;
	ros::NodeHandle nh("~");

	nh.getParam("scale", initial_config.scale);
	nh.getParam("Kp", initial_config.Kp);
	nh.getParam("Ki", initial_config.Ki);
	nh.getParam("Kd", initial_config.Kd);
	nh.getParam("db", initial_config.db);
	nh.getParam("st", initial_config.st);
	nh.getParam("wu", initial_config.wu);

	// Announce the configuration to the server
	dynamic_reconfigure::Client<bridge::DriveConfig> dyn_client(rover + "_SBRIDGE");
	dyn_client.setConfiguration(initial_config);

	cout << "Initial configuration sent." << endl;
}
