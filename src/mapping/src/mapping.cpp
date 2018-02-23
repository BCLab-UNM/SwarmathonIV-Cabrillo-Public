/*
 * mapping.cpp
 *
 *  Created on: Jul 2, 2017
 *      Author: maximus
 */

#include <iostream>
#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>

#include <swarmie_msgs/Obstacle.h>
#include <mapping/FindTarget.h>
#include <mapping/GetMap.h>

using namespace std;

string rover;

grid_map::GridMap rover_map;

ros::Publisher obstaclePublish;
ros::Publisher rover_map_publisher;

geometry_msgs::Pose2D currentLocation;
tf::TransformListener *cameraTF;

double collisionDistance = 0.6; //meters the ultrasonic detectors will flag obstacles
unsigned int obstacle_status;

double poseToYaw(const geometry_msgs::Pose &pose) {
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void publishRoverMap() {
	if (rover_map_publisher.getNumSubscribers() > 0) {
		ros::Time time = ros::Time::now();
		grid_map_msgs::GridMap message;
		rover_map.setTimestamp(time.toNSec());
		grid_map::GridMapRosConverter::toMessage(rover_map, message);
		rover_map_publisher.publish(message);
	}
}

/* sonarHandler() - Called when there's new data from the sonar array.
 *
 * This does two things:
 *
 * 1. Check the sonar distances to see if they are <= to the collision distance.
 *   If so send an obstacle message so that the rover stops.
 *
 * 2. Update the obstacle maps based on current sonar readings.
 *
 */
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {

	static unsigned int prev_status = 0;
	unsigned int next_status = 0;
    double sonar_depth = 0.1;  // limit view_poly to 10cm past measured ranges

	// Update the timestamp in the Obstacle map.
	rover_map.setTimestamp(ros::Time::now().toNSec());

	// Calculate the obstacle status.
	if (sonarLeft->range < collisionDistance && sonarCenter->range < collisionDistance) {
		next_status |= swarmie_msgs::Obstacle::SONAR_LEFT;
		next_status |= swarmie_msgs::Obstacle::SONAR_CENTER;
	}
	if (sonarRight->range < collisionDistance && sonarCenter->range < collisionDistance) {
		next_status |= swarmie_msgs::Obstacle::SONAR_RIGHT;
		next_status |= swarmie_msgs::Obstacle::SONAR_CENTER;
	}
	if (sonarCenter->range < 0.12) {
		//block in front of center ultrasound.
		next_status |= swarmie_msgs::Obstacle::SONAR_BLOCK;
	}

	grid_map::Polygon view_poly;
	view_poly.setFrameId(rover_map.getFrameId());

	view_poly.addVertex(grid_map::Position(currentLocation.x, currentLocation.y));

	// Left sonar
	view_poly.addVertex(
			grid_map::Position(currentLocation.x + (sonarLeft->range + sonar_depth) * cos(currentLocation.theta + M_PI_4),
					currentLocation.y + (sonarLeft->range + sonar_depth) * sin(currentLocation.theta + M_PI_4)
			));

	// Center sonar
	view_poly.addVertex(
			grid_map::Position(currentLocation.x + (sonarCenter->range + sonar_depth) * cos(currentLocation.theta),
					currentLocation.y + (sonarCenter->range + sonar_depth) * sin(currentLocation.theta)
			));

	// Right sonar
	view_poly.addVertex(
			grid_map::Position(currentLocation.x + (sonarRight->range + sonar_depth) * cos(currentLocation.theta - M_PI_4),
					currentLocation.y + (sonarRight->range + sonar_depth) * sin(currentLocation.theta - M_PI_4)
			));

	// Increase the "obstacleness" of the viewable area.
	for (grid_map::PolygonIterator iterator(rover_map, view_poly);
	      !iterator.isPastEnd(); ++iterator) {
		double val = rover_map.at("obstacle", *iterator);
		if (isnan(val)) {
			val = 0.5;
		}
		val += 0.01;
		if (val > 1)
			val = 1;
	    rover_map.at("obstacle", *iterator) = val;
	  }


	grid_map::Polygon clear_poly;
	clear_poly.setFrameId(rover_map.getFrameId());

	clear_poly.addVertex(grid_map::Position(currentLocation.x, currentLocation.y));

	// Left sonar
	clear_poly.addVertex(
			grid_map::Position(currentLocation.x + sonarLeft->range * cos(currentLocation.theta + M_PI_4),
					currentLocation.y + sonarLeft->range * sin(currentLocation.theta + M_PI_4)
			));

	// Center sonar
	clear_poly.addVertex(
			grid_map::Position(currentLocation.x + sonarCenter->range * cos(currentLocation.theta),
					currentLocation.y + sonarCenter->range * sin(currentLocation.theta)
			));

	// Right sonar
	clear_poly.addVertex(
			grid_map::Position(currentLocation.x + sonarRight->range * cos(currentLocation.theta - M_PI_4),
					currentLocation.y + sonarRight->range * sin(currentLocation.theta - M_PI_4)
			));

	// Clear the area that's clear in sonar.
	for (grid_map::PolygonIterator iterator(rover_map, clear_poly);
	      !iterator.isPastEnd(); ++iterator) {
		double val = rover_map.at("obstacle", *iterator);
		if (!isnan(val)) {
			val -= 0.05;
			if (val < 0)
				val = 0;
		}
	    rover_map.at("obstacle", *iterator) = val;
	  }

	// Publish the obstacle message if there's an update to it.
	if (next_status != prev_status) {
		swarmie_msgs::Obstacle msg;
		msg.msg = next_status;
		msg.mask = swarmie_msgs::Obstacle::IS_SONAR;
		obstaclePublish.publish(msg);
	}

	prev_status = next_status;

	publishRoverMap();
}

/* sonarHandler() - Called when there's new Apriltag detection data.
 *
 * This does two things:
 *
 * 1. Send and obstacle message if there's a visible tag.
 *
 * 2. Update the target maps based on current detections.
 *
 */
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
	static unsigned int prev_status = 0;
	unsigned int next_status = 0;

	// Update the timestamp in the target map.
	rover_map.setTimestamp(ros::Time::now().toNSec());

	if (message->detections.size() > 0) {
		for (int i=0; i<message->detections.size(); i++) {

			if (message->detections[i].id == 0) {
				next_status |= swarmie_msgs::Obstacle::TAG_TARGET;
			}
			else if (message->detections[i].id == 256) {
				next_status |= swarmie_msgs::Obstacle::TAG_HOME;
			}
		}
	}

	grid_map::Position pos (currentLocation.x, currentLocation.y);
	grid_map::Index map_index;
	rover_map.getIndex(pos, map_index);

	grid_map::Polygon view_poly;
	view_poly.setFrameId(rover_map.getFrameId());

	view_poly.addVertex(grid_map::Position(currentLocation.x, currentLocation.y));

	view_poly.addVertex(
			grid_map::Position(currentLocation.x + 1 * cos(currentLocation.theta + M_PI_4),
					currentLocation.y + 1 * sin(currentLocation.theta + M_PI_4)
			));

	view_poly.addVertex(
			grid_map::Position(currentLocation.x + 1 * cos(currentLocation.theta),
					currentLocation.y + 1 * sin(currentLocation.theta)
			));

	view_poly.addVertex(
			grid_map::Position(currentLocation.x + 1 * cos(currentLocation.theta - M_PI_4),
					currentLocation.y + 1 * sin(currentLocation.theta - M_PI_4)
			));


	for (grid_map::PolygonIterator iterator(rover_map, view_poly);
	      !iterator.isPastEnd(); ++iterator) {

		double home_val = rover_map.at("home", *iterator);
		if (isnan(home_val)) {
			home_val = 0;
		}
		if (next_status & swarmie_msgs::Obstacle::TAG_HOME) {
			home_val += 1;
			if (home_val > 1)
				home_val = 1;
		}
		else {
			home_val -= 1;
			if (home_val < 0)
				home_val = 0;
		}
		rover_map.at("home", *iterator) = home_val;

		double target_val = rover_map.at("target", *iterator);
		if (isnan(target_val)) {
			target_val = 0;
		}
		if (next_status & swarmie_msgs::Obstacle::TAG_TARGET) {
			target_val += 1;
			if (target_val > 1)
				target_val = 1;
		}
		else {
			target_val -= 1;
			if (target_val < 0)
				target_val = 0;
		}
		rover_map.at("target", *iterator) = target_val;

	}

	if (next_status != prev_status) {
		swarmie_msgs::Obstacle msg;
		msg.msg = next_status;
		msg.mask = swarmie_msgs::Obstacle::IS_VISION;
		obstaclePublish.publish(msg);
	}

	prev_status = next_status;

	publishRoverMap();
}

/* odometryHandler() - Called when new odometry data is available.
 *
 * The target map is high resolution and doesn't cover the whole arena.
 * Use odometry data to recenter the map on top of the rover. Data that
 * "falls off" is forgotten.
 */
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
    double x = message->pose.pose.position.x;
    double y = message->pose.pose.position.y;

    // Store the current location so we can use it in other places.
    currentLocation.x = x;
    currentLocation.y = y;
    currentLocation.theta = poseToYaw(message->pose.pose);
}

/* Python API
 *
 * get_map() - Return a view of the rover's grid_map.
 *
 * The service definition is in:
 * 	srv/GetMap.srv
 *
 */
bool get_map(mapping::GetMap::Request &req, mapping::GetMap::Response &rsp) {
	grid_map::GridMapRosConverter::toMessage(rover_map, rsp.map);
	return true;
}

int main(int argc, char **argv) {

	char host[128];
    gethostname(host, sizeof (host));
    string hostname(host);

    if (argc >= 2) {
        rover = argv[1];
        cout << "Welcome to the world of tomorrow " << rover
             << "!  Mapping node started." << endl;
    } else {
        rover = hostname;
        cout << "No Name Selected. Default is: " << rover << endl;
    }

    ros::init(argc, argv, (rover + "_MAP"));
    ros::NodeHandle mNH;

    obstacle_status = swarmie_msgs::Obstacle::PATH_IS_CLEAR;

    // Transform Listener
    //
    // C++ Tutorial Here:
    // 		http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
    cameraTF = new tf::TransformListener(ros::Duration(10));

    // Subscribers
    //
    // C++ Tutorial Here:
    // 		http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
    //
    ros::Subscriber odomSubscriber = mNH.subscribe((rover + "/odom/filtered"), 10, odometryHandler);
    ros::Subscriber targetSubscriber = mNH.subscribe((rover + "/targets"), 10, targetHandler);

    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, (rover + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, (rover + "/sonarCenter"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, (rover + "/sonarRight"), 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    //	Publishers

    obstaclePublish = mNH.advertise<swarmie_msgs::Obstacle>((rover + "/obstacle"), 1, true);

    rover_map_publisher = mNH.advertise<grid_map_msgs::GridMap>(rover + "/map", 1, false);

    // Services
    //
    // This is the API into the Python control code
    //
    ros::ServiceServer omap = mNH.advertiseService(rover + "/map/get_map", get_map);

    // Initialize the maps.
    rover_map = grid_map::GridMap({"obstacle", "target", "home"});
    rover_map.setFrameId(rover + "/odom");
    rover_map.setGeometry(grid_map::Length(25, 25), 0.5);

    ros::spin();
    return 0;
}
