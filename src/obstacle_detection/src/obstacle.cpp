#include <ros/ros.h>

//ROS libraries
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/service.h>

//ROS messages
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

#include "obstacle_detection/Obstacle.h"

using namespace std;

//Globals
double collisionDistance = 0.6; //meters the ultrasonic detectors will flag obstacles
string publishedName;
char host[128];

float heartbeat_publish_interval = 2;

//Publishers
ros::Publisher obstaclePublish;
ros::Publisher heartbeatPublisher;

//Subscribers
ros::Subscriber targetSubscriber;

//Timers
ros::Timer publish_heartbeat_timer;

unsigned int obstacle_status;

//Callback handlers
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);

int main(int argc, char** argv) {
    gethostname(host, sizeof (host));
    string hostname(host);
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "! Obstacle module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No name selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_OBSTACLE"));
    ros::NodeHandle oNH;
    
    obstaclePublish = oNH.advertise<obstacle_detection::Obstacle>((publishedName + "/obstacle"), 1, true);
    heartbeatPublisher = oNH.advertise<std_msgs::String>((publishedName + "/obstacle/heartbeat"), 1, true);
    
    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(oNH, (publishedName + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(oNH, (publishedName + "/sonarCenter"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(oNH, (publishedName + "/sonarRight"), 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    targetSubscriber = oNH.subscribe((publishedName + "/targets"), 1, targetHandler);

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    publish_heartbeat_timer = oNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

    obstacle_status = obstacle_detection::Obstacle::PATH_IS_CLEAR;

    ros::spin();

    return EXIT_SUCCESS;
}

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
	static unsigned int prev_status = 0;
	unsigned int next_status = 0;

	for (int i=0; i<message->detections.size(); i++) {
		if (message->detections[i].id == 0)
			next_status |= obstacle_detection::Obstacle::TAG_TARGET;
		else if (message->detections[i].id == 256)
			next_status |= obstacle_detection::Obstacle::TAG_HOME;
	}

	if (next_status != prev_status) {
		obstacle_detection::Obstacle msg;
		msg.msg = next_status;
		msg.mask = obstacle_detection::Obstacle::IS_VISION;
		obstaclePublish.publish(msg);
	}

	prev_status = next_status;
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {

	static unsigned int prev_status = 0;
	unsigned int next_status = 0;

	// TODO: Implement filtering for sonar.
	
	if (sonarLeft->range < collisionDistance) {
		next_status |= obstacle_detection::Obstacle::SONAR_LEFT;
	}
	if (sonarRight->range < collisionDistance) {
		next_status |= obstacle_detection::Obstacle::SONAR_RIGHT;
	}
	if (sonarCenter->range < collisionDistance) {
		next_status |= obstacle_detection::Obstacle::SONAR_CENTER;
	}
	if (sonarCenter->range < 0.12) {
		//block in front of center ultrasound.
		next_status |= obstacle_detection::Obstacle::SONAR_BLOCK;
	}

	if (next_status != prev_status) {
		obstacle_detection::Obstacle msg;
		msg.msg = next_status;
		msg.mask = obstacle_detection::Obstacle::IS_SONAR;
		obstaclePublish.publish(msg);
	}

	prev_status = next_status;
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
}
