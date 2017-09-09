#include <signal.h>
#include <iostream>
#include <math.h>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/client.h>
#include "tf/transform_datatypes.h"

#include "pid.h"
#include <bridge/DriveConfig.h>

using namespace std;

ros::Publisher heartbeatPublisher;
ros::Publisher skidsteerPublisher;
ros::Publisher infoLogPublisher;

ros::Publisher debugPIDPublisher;
geometry_msgs::Twist dbT;

ros::Subscriber driveControlSubscriber;
ros::Subscriber odomSubscriber;

ros::Timer heartbeatTimer;
//ros::Timer actionTimer;

string rover;

// Immobilize robot until the first PID configuration.
PID left_wheel(0, 0, 0, 0, 1, -1, 0, -1);
PID right_wheel(0, 0, 0, 0, 1, -1, 0, -1);

double sp_linear = 0, sp_delta = 0;
int drive_mode = DRIVE_MODE_STOP;

const double wheelBase = 0.278; //distance between left and right wheels (in M)

void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
}

void cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
	sp_linear = message->linear.x;   // M/s
	sp_delta = (message->angular.z * M_PI * wheelBase) / 2; // Delta M/s (/2)

	int next_mode = round(message->angular.y);
	if (drive_mode == DRIVE_MODE_STOP && next_mode == DRIVE_MODE_PID) {

		/* Feed forward controller:
		 *
		 * In the simulator this is trivial because the output is
		 * velocity, not motor control voltage.
		 */

		left_wheel.feedforward(sp_linear - sp_delta, 0.5);
		right_wheel.feedforward(sp_linear + sp_delta, 0.5);
	}

	drive_mode = next_mode;
}

void odomHandler(const nav_msgs::Odometry::ConstPtr& message) {
	double linear_velocity, delta_velocity;

	/*
	tf::Quaternion quat;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(message->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	linear_velocity = message->twist.twist.linear.x * sin(yaw)
		+ message->twist.twist.linear.y * cos(yaw);
	 */

	linear_velocity = message->twist.twist.linear.x;
    delta_velocity = (message->twist.twist.angular.z * M_PI * wheelBase) / 2;

	geometry_msgs::Twist velocity;

	if (drive_mode == DRIVE_MODE_STOP) {
		velocity.linear.x = 0;
		velocity.angular.z = 0;
		left_wheel.reset();
		right_wheel.reset();
	}
	else {
		double now = (double(message->header.stamp.sec) * 10e9 + message->header.stamp.nsec) / 10e9;

		double left_out = left_wheel.step(sp_linear - sp_delta,
				linear_velocity - delta_velocity, now);
		double right_out = right_wheel.step(sp_linear + sp_delta,
				linear_velocity + delta_velocity, now);

		velocity.linear.x = (left_out + right_out) / 2;
		velocity.angular.z = 2 * (right_out - left_out) / M_PI;

		// Debugging: Report PID performance for tuning.
		// Output of the PID is in Linear:
		dbT.linear.x = left_out;
		dbT.linear.y = right_out;
		dbT.linear.z = sp_linear;

		// Feedback function is in Angular:
		dbT.angular.x = sp_delta;
		dbT.angular.y = linear_velocity;
		dbT.angular.z = delta_velocity;
		debugPIDPublisher.publish(dbT);

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

    heartbeatPublisher = sNH.advertise<std_msgs::String>((rover + "/bridge/heartbeat"), 1, false);
    skidsteerPublisher = sNH.advertise<geometry_msgs::Twist>((rover + "/skidsteer"), 10);
    infoLogPublisher = sNH.advertise<std_msgs::String>("/infoLog", 1, true);

    debugPIDPublisher = sNH.advertise<geometry_msgs::Twist>((rover + "/bridge/debugPID"), 1, false);

    heartbeatTimer = sNH.createTimer(ros::Duration(2), publishHeartBeatTimerEventHandler);
    //actionTimer = sNH.createTimer(ros::Duration(0.1), doPIDs);

    // configure dynamic reconfiguration
    dynamic_reconfigure::Server<bridge::DriveConfig> config_server;
    dynamic_reconfigure::Server<bridge::DriveConfig>::CallbackType f;
    f = boost::bind(&reconfigure, _1, _2);
    config_server.setCallback(f);

    boost::thread t(initialconfig);
    ros::spin();

	return EXIT_SUCCESS;
}
