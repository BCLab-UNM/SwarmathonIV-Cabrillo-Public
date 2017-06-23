#include <signal.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>

#include "pid.h"
#include <bridge/DriveConfig.h>

using namespace std;

ros::Publisher heartbeatPublisher;
ros::Publisher skidsteerPublisher;
ros::Publisher infoLogPublisher;

ros::Subscriber driveControlSubscriber;
ros::Subscriber odomSubscriber;

ros::Timer heartbeatTimer;
ros::Timer actionTimer;

string rover;

// Immobilize robot until the first PID configuration.
PID left_wheel(0, 0, 0, 0, 1, -1, 0, -1);
PID right_wheel(0, 0, 0, 0, 1, -1, 0, -1);

double linear_velocity = 0, angular_velocity = 0;
double sp_linear = 0, sp_angular = 0;
int drive_mode = DRIVE_MODE_STOP;

void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
}

void cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
	sp_linear = message->linear.x;
	sp_angular = message->angular.z;
	drive_mode = round(message->angular.y);
}

void odomHandler(const nav_msgs::Odometry::ConstPtr& message) {
    linear_velocity = message->twist.twist.linear.x;
    angular_velocity = message->twist.twist.angular.z;
}

void doPIDs(const ros::TimerEvent& event) {

	geometry_msgs::Twist velocity;

	if (drive_mode == DRIVE_MODE_STOP) {
		velocity.linear.x = 0;
		velocity.angular.z = 0;
		left_wheel.reset();
		right_wheel.reset();
	}
	else {
		double now = event.current_real.toSec();

		double left_out = left_wheel.step(sp_linear - sp_angular, linear_velocity - angular_velocity, now);
		double right_out = right_wheel.step(sp_linear + sp_angular, linear_velocity + angular_velocity, now);

		velocity.linear.x = left_out + right_out;
		velocity.angular.z = right_out - left_out;
	}
    skidsteerPublisher.publish(velocity);
}

void reconfigure(bridge::DriveConfig &cfg, uint32_t level) {
	double p, i, d, db, st, wu;
	p = cfg.groups.pid.Kp * cfg.groups.pid.scale;
	i = cfg.groups.pid.Ki * cfg.groups.pid.scale;
	d = cfg.groups.pid.Kd * cfg.groups.pid.scale;
	db = cfg.groups.pid.db;
	st = cfg.groups.pid.st;
	wu = cfg.groups.pid.wu;

	left_wheel.reconfig(p, i, d, db, st, wu);
	right_wheel.reconfig(p, i, d, db, st, wu);

	std_msgs::String msg;
	msg.data = "PID reconfigure is done.";
	infoLogPublisher.publish(msg);
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

    heartbeatPublisher = sNH.advertise<std_msgs::String>((rover + "/sbridge/heartbeat"), 1, false);
    skidsteerPublisher = sNH.advertise<geometry_msgs::Twist>((rover + "/skidsteer"), 10);
    infoLogPublisher = sNH.advertise<std_msgs::String>("/infoLog", 1, true);

    heartbeatTimer = sNH.createTimer(ros::Duration(2), publishHeartBeatTimerEventHandler);
    actionTimer = sNH.createTimer(ros::Duration(0.1), doPIDs);

    // configure dynamic reconfiguration
    dynamic_reconfigure::Server<bridge::DriveConfig> config_server;
    dynamic_reconfigure::Server<bridge::DriveConfig>::CallbackType f;
    f = boost::bind(&reconfigure, _1, _2);
    config_server.setCallback(f);

    ros::spin();

	return EXIT_SUCCESS;
}
