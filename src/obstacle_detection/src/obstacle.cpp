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
#include "obstacle_detection/DetectionMask.h"

using namespace std;

//Globals
double collisionDistance = 0.6; //meters the ultrasonic detectors will flag obstacles
string publishedName;
char host[128];

float heartbeat_publish_interval = 2;

//Publishers
ros::Publisher obstaclePublish;
ros::Publisher heartbeatPublisher;

//Timers
ros::Timer publish_heartbeat_timer;

unsigned int obstacle_mask;
unsigned int obstacle_status;

//Callback handlers
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
bool setObstacleMask(obstacle_detection::DetectionMask::Request &req, obstacle_detection::DetectionMask::Response &resp);
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
    
    obstaclePublish = oNH.advertise<obstacle_detection::Obstacle>((publishedName + "/obstacle"), 10);
    heartbeatPublisher = oNH.advertise<std_msgs::String>((publishedName + "/obstacle/heartbeat"), 1, true);
    
    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(oNH, (publishedName + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(oNH, (publishedName + "/sonarCenter"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(oNH, (publishedName + "/sonarRight"), 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    oNH.subscribe((publishedName + "/targets"), 10, targetHandler);

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    publish_heartbeat_timer = oNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

    obstacle_mask = obstacle_detection::Obstacle::SONAR_LEFT
    		| obstacle_detection::Obstacle::SONAR_RIGHT
			| obstacle_detection::Obstacle::SONAR_CENTER
			| obstacle_detection::Obstacle::SONAR_BLOCK
			| obstacle_detection::Obstacle::TAG_SEEN;

    obstacle_status = obstacle_detection::Obstacle::PATH_IS_CLEAR;

    ros::ServiceServer srv = oNH.advertiseService(publishedName + "/obstacleMask", setObstacleMask);

    ros::spin();

    return EXIT_SUCCESS;
}

bool setObstacleMask(obstacle_detection::DetectionMask::Request &req, obstacle_detection::DetectionMask::Response &resp) {
	resp.rval = obstacle_mask;
	obstacle_mask = req.mask;
	cout << "Obstacle mask set to: " << obstacle_status << " was: " << resp.rval << endl;
	return true;
}

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
	obstacle_status &= ~obstacle_detection::Obstacle::IS_VISION;
	if (message->detections.size() > 0) {
		obstacle_status |= obstacle_detection::Obstacle::TAG_SEEN;
    }
	obstacle_status &= obstacle_mask;
	if (obstacle_status & obstacle_detection::Obstacle::IS_VISION) {
		obstacle_detection::Obstacle msg;
		msg.msg = obstacle_status;
		obstaclePublish.publish(msg);
	}
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {

	// TODO: Implement filtering for sonar.

	obstacle_status &= ~obstacle_detection::Obstacle::IS_SONAR;
	
	if (sonarLeft->range < collisionDistance) {
		obstacle_status |= obstacle_detection::Obstacle::SONAR_LEFT;
	}
	if (sonarRight->range < collisionDistance) {
		obstacle_status |= obstacle_detection::Obstacle::SONAR_RIGHT;
	}
	if (sonarCenter->range < collisionDistance) {
		obstacle_status |= obstacle_detection::Obstacle::SONAR_CENTER;
	}
	if (sonarCenter->range < 0.12) {
		//block in front of center ultrasound.
		obstacle_status |= obstacle_detection::Obstacle::SONAR_BLOCK;
	}

	cout << "Obstacle status: " << obstacle_status << " mask: " << obstacle_mask << endl;

	// Apply the mask
	obstacle_status &= obstacle_mask;

	if (obstacle_status & obstacle_detection::Obstacle::IS_SONAR) {
		obstacle_detection::Obstacle msg;
		msg.msg = obstacle_status;
		obstaclePublish.publish(msg);
	}
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
}
