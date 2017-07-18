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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;

string rover;

grid_map::GridMap rover_map;
ros::Publisher map_publisher;
geometry_msgs::Pose2D currentLocation;

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {
/*
	doSweep(sonarCenter->range, M_PI);
	doSweep(sonarLeft->range, M_PI + M_PI_4);
	doSweep(sonarRight->range, M_PI - M_PI_4);
	*/
	grid_map::Polygon view_poly;
	view_poly.setFrameId(rover_map.getFrameId());

	view_poly.addVertex(grid_map::Position(0, 0));

	// Left sonar
	view_poly.addVertex(
			grid_map::Position(3 * cos(currentLocation.theta + M_PI + M_PI_4),
					3 * sin(currentLocation.theta + M_PI + M_PI_4)
			));

	// Center sonar
	view_poly.addVertex(
			grid_map::Position(3 * cos(currentLocation.theta + M_PI),
					3 * sin(currentLocation.theta + M_PI)
			));

	// Right sonar
	view_poly.addVertex(
			grid_map::Position(3 * cos(currentLocation.theta + M_PI - M_PI_4),
					3 * sin(currentLocation.theta + M_PI - M_PI_4)
			));

	// Increase the "obstacleness" of the viewable area.
	for (grid_map::PolygonIterator iterator(rover_map, view_poly);
	      !iterator.isPastEnd(); ++iterator) {
		double val = rover_map.at("obstacles", *iterator);
		val += 0.01;
		if (val > 1)
			val = 1;
	    rover_map.at("obstacles", *iterator) = val;
	  }


	grid_map::Polygon clear_poly;
	clear_poly.setFrameId(rover_map.getFrameId());

	clear_poly.addVertex(grid_map::Position(0, 0));

	// Left sonar
	clear_poly.addVertex(
			grid_map::Position(sonarLeft->range * cos(currentLocation.theta + M_PI + M_PI_4),
					sonarLeft->range * sin(currentLocation.theta + M_PI + M_PI_4)
			));

	// Center sonar
	clear_poly.addVertex(
			grid_map::Position(sonarCenter->range * cos(currentLocation.theta + M_PI),
					sonarCenter->range * sin(currentLocation.theta + M_PI)
			));

	// Right sonar
	clear_poly.addVertex(
			grid_map::Position(sonarRight->range * cos(currentLocation.theta + M_PI - M_PI_4),
					sonarRight->range * sin(currentLocation.theta + M_PI - M_PI_4)
			));

	// Clear the area that's clear in sonar.
	for (grid_map::PolygonIterator iterator(rover_map, clear_poly);
	      !iterator.isPastEnd(); ++iterator) {
		double val = rover_map.at("obstacles", *iterator);
		val -= 0.05;
		if (val < 0)
			val = 0;
	    rover_map.at("obstacles", *iterator) = val;
	  }

}

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
	for (int i=0; i<message->detections.size(); i++) {
		// Tag positions are relative to base_link. Add Odometry to place them
		// into the map frame. Right?
		geometry_msgs::Point p = message->detections[i].pose.pose.position;
		grid_map::Position pos(p.x + currentLocation.x, p.y + currentLocation.y);
		grid_map::Index ind;
		rover_map.getIndex(pos, ind);

		rover_map.at("targets", ind) = 1;
	}
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
    double x = message->pose.pose.position.x;
    double y = message->pose.pose.position.y;

    grid_map::Position newPos(x, y);
    rover_map.setPosition(newPos);

    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.x = x;
    currentLocation.y = y;
    currentLocation.theta = yaw;
}

void publishMap(const ros::TimerEvent& event) {
    ros::Time time = ros::Time::now();
	rover_map.setTimestamp(time.toNSec());
	grid_map_msgs::GridMap message;
	grid_map::GridMapRosConverter::toMessage(rover_map, message);
	map_publisher.publish(message);
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

    // Subscribers

    ros::Subscriber odomSubscriber = mNH.subscribe((rover+ "/odom/filtered"), 10, odometryHandler);
    ros::Subscriber targetSubscriber = mNH.subscribe((rover + "/targets"), 10, targetHandler);

    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, (rover + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, (rover + "/sonarCenter"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, (rover + "/sonarRight"), 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    //	Publishers

    map_publisher = mNH.advertise<grid_map_msgs::GridMap>(rover + "/grid_map", 1, true);

    // Initialize the map.
    rover_map = grid_map::GridMap({"obstacles", "targets"});
    rover_map.setFrameId(rover + "/map");
    rover_map.setGeometry(grid_map::Length(25, 25), 0.5);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
      rover_map.getLength().x(), rover_map.getLength().y(),
      rover_map.getSize()(0), rover_map.getSize()(1));

    // All cells should look like "fog"
    for (grid_map::GridMapIterator it(rover_map); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      rover_map.getPosition(*it, position);
      rover_map.at("obstacles", *it) = 0.5;
      rover_map.at("targets", *it) = 0.5;
    }

    // Publish map updates.
    ros::Timer map_update = mNH.createTimer(ros::Duration(0.2), publishMap);

    ros::spin();
    return 0;
}


