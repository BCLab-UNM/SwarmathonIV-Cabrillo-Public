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
#include <bridge/pidConfig.h>

using namespace std;

ros::Publisher heartbeatPublisher;
ros::Publisher skidsteerPublisher;
ros::Publisher infoLogPublisher;

ros::Subscriber driveControlSubscriber;

ros::Timer heartbeatTimer;

string rover;

const double wheelBase = 0.278; //distance between left and right wheels (in M)

void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
}

void cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
	geometry_msgs::Twist velocity;
	velocity.linear.x = message->linear.x;
	velocity.angular.z = message->angular.z;
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

    heartbeatPublisher = sNH.advertise<std_msgs::String>((rover + "/bridge/heartbeat"), 1, false);
    skidsteerPublisher = sNH.advertise<geometry_msgs::Twist>((rover + "/skidsteer"), 10);
    infoLogPublisher = sNH.advertise<std_msgs::String>("/infoLog", 1, true);

    heartbeatTimer = sNH.createTimer(ros::Duration(2), publishHeartBeatTimerEventHandler);

    ros::spin();

	return EXIT_SUCCESS;
}
