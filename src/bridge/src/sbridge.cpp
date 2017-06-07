#include <signal.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>

#include "pid.h"

using namespace std;

ros::Publisher heartbeatPublisher;
ros::Publisher skidsteerPublisher;
ros::Publisher infoLogPublisher;

ros::Subscriber driveControlSubscriber;
ros::Subscriber robotPositionSubscriber;
ros::Subscriber odomSubscriber;

ros::Timer heartbeatTimer;
ros::Timer actionTimer;

string rover;

PID linear(0.1, 0, 0, 0, 1, -1);
PID angular(0.1, 0, 0, 0, 1, -1);

double linear_velocity = 0, angular_velocity = 0;
double sp_linear = 0, sp_angular = 0;

void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
}

void cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
	sp_linear = message->linear.x;
	sp_angular = message->angular.z;
}

/*
void positionHandler(const gazebo_msgs::ModelStates::ConstPtr& message) {
	int rover_pos = -1;

	for (int i=0; i < message->name.size(); i++) {
		if (message->name[i] == rover) {
			rover_pos = i;
			break;
		}
	}

	if (rover_pos < 0)
		return;

	geometry_msgs::Point position = message->pose[rover_pos].position;
	geometry_msgs::Quaternion orientation = message->pose[rover_pos].orientation;

    double roll, pitch, yaw;
    tf::Matrix3x3 m(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    m.getRPY(roll, pitch, yaw);

    pos_x = position.x;
    pos_y = position.y;
    pos_theta = yaw;
}
*/

void odomHandler(const nav_msgs::Odometry::ConstPtr& message) {
	static double lastx = 0;
	static double lasty = 0;
	static double lasttheta = 0;

    linear_velocity = message->twist.twist.linear.x;
    angular_velocity = message->twist.twist.angular.z;
}

void doPIDs(const ros::TimerEvent& event) {
    double linear_out = linear.step(sp_linear, linear_velocity);
    double angular_out = angular.step(sp_angular, angular_velocity);

	geometry_msgs::Twist velocity;
    velocity.linear.x = linear_out;
    velocity.angular.z = angular_out;

    cout << "l: " << linear_out << " a: " << angular_out << endl;
    skidsteerPublisher.publish(velocity);
}

int main(int argc, char **argv) {
    sleep(10);

    char host[128];
    gethostname(host, sizeof (host));
    string hostname(host);

    if (argc >= 2) {
        rover = argv[1];
        cout << "Welcome to the world of tomorrow " << rover << "!  SBridge module started." << endl;
    } else {
    	cout << "Specify rover name." << endl;
    	return 1;
    }

    ros::init(argc, argv, (rover + "_SBRIDGE"), ros::init_options::NoSigintHandler);

    ros::NodeHandle sNH;

    driveControlSubscriber = sNH.subscribe((rover + "/driveControl"), 10, &cmdHandler);
    odomSubscriber = sNH.subscribe((rover + "/odom"), 1, &odomHandler);
    //robotPositionSubscriber = sNH.subscribe("/gazebo/model_states", 1, &positionHandler);

    heartbeatPublisher = sNH.advertise<std_msgs::String>((rover + "/sbridge/heartbeat"), 1, false);
    skidsteerPublisher = sNH.advertise<geometry_msgs::Twist>((rover + "/skidsteer"), 10);
    infoLogPublisher = sNH.advertise<std_msgs::String>("/infoLog", 1, true);

    heartbeatTimer = sNH.createTimer(ros::Duration(2), publishHeartBeatTimerEventHandler);
    actionTimer = sNH.createTimer(ros::Duration(0.1), doPIDs);

    ros::spin();

	return EXIT_SUCCESS;
}
