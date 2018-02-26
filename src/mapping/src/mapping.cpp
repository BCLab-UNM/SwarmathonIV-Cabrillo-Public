/*
 * mapping.cpp
 *
 *  Created on: Jul 2, 2017
 *      Author: maximus
 */

#include <iostream>
#include <string>
#include <cmath>
#include <map>
#include <set>
#include <array>
#include <queue>

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
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>

#include <swarmie_msgs/Obstacle.h>
#include <mapping/FindTarget.h>
#include <mapping/GetMap.h>

using namespace std;

string rover;

grid_map::GridMap obstacle_map;
grid_map::GridMap target_map;

ros::Publisher obstaclePublish;
ros::Publisher obstacle_map_publisher;
ros::Publisher target_map_publisher;
ros::Publisher path_publisher;

geometry_msgs::Pose2D currentLocation;
tf::TransformListener *cameraTF;

double collisionDistance = 0.6; //meters the ultrasonic detectors will flag obstacles
unsigned int obstacle_status;


/*
 * Data structures and A* Search adapted from:
 * https://www.redblobgames.com/pathfinding/a-star/introduction.html
 * https://www.redblobgames.com/pathfinding/a-star/implementation.html
 *
 * Sample code from https://www.redblobgames.com/pathfinding/a-star/
 *
 * Copyright 2014 Red Blob Games <redblobgames@gmail.com>
 * Feel free to use this code in your own projects, including commercial projects
 * License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
 */

struct GridLocation {
	int x, y;
};

// Helpers for GridLocation

bool operator == (GridLocation a, GridLocation b) {
	return a.x == b.x && a.y == b.y;
}

bool operator != (GridLocation a, GridLocation b) {
	return !(a == b);
}

bool operator < (GridLocation a, GridLocation b) {
	return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

bool in_bounds(grid_map::GridMap& map, GridLocation location) {
        int width = map.getSize()(1);
		int height = map.getSize()(0);
		return 0 <= location.x && location.x < width
			   && 0 <= location.y && location.y < height;
}

const std::array<GridLocation, 4> DIRECTIONS =
		{GridLocation{1, 0}, GridLocation{0, -1}, GridLocation{-1, 0}, GridLocation{0, 1}};

std::vector<GridLocation> neighbors(grid_map::GridMap& map, GridLocation location) {
    std::vector<GridLocation> results;

    for (GridLocation direction : DIRECTIONS) {
        GridLocation next{location.x + direction.x, location.y + direction.y};
		// removed passable() for now. Might want it back if you do 8-connected grid
        if (in_bounds(map, next)) {
            results.push_back(next);
        }
    }

    if ((location.x + location.y) % 2 == 0) {
        // aesthetic improvement on square grids
        std::reverse(results.begin(), results.end());
    }

    return results;
}

// to_node should be valid position on the grid. Not necessarily with a finite
// value, though.
double cost(grid_map::GridMap& map, GridLocation to_node) {
	double cost = 0.0;
	// I think these are the correct indexes for x, y coords
	grid_map::Index index;
	index(0) = to_node.y;
	index(1) = to_node.x;

	if (!map.isValid(index, "obstacles")) {
		cost = 0.0;
	} else {
		cost = map.at("obstacles", index);
	}

    return cost;
}

template<typename T, typename priority_t>
struct PriorityQueue {
	typedef std::pair<priority_t, T> PQElement;
	std::priority_queue<PQElement, std::vector<PQElement>,
			std::greater<PQElement>> elements;

	inline bool empty() const {
		return elements.empty();
	}

	inline void put(T item, priority_t priority) {
		elements.emplace(priority, item);
	}

	T get() {
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	}
};

inline double heuristic(GridLocation a, GridLocation b) {
	return abs(a.x - b.x) + abs(a.y - b.y);
}

void a_star_search(
		grid_map::GridMap& map,
        GridLocation start,
        GridLocation goal,
        std::map<GridLocation, GridLocation>& came_from,
        std::map<GridLocation, double>& cost_so_far) {

	PriorityQueue<GridLocation, double> frontier;
	frontier.put(start, 0);

	came_from[start] = start;
	cost_so_far[start] = 0;

	while (!frontier.empty()) {
		GridLocation current = frontier.get();

		if (current == goal) {
			break;
		}

		for (GridLocation next : neighbors(map, current)) {
			double new_cost = cost_so_far[current] + cost(map, next);
			if (cost_so_far.find(next) == cost_so_far.end()
				|| new_cost < cost_so_far[next]) {
				cost_so_far[next] = new_cost;
				double priority = new_cost + heuristic(next, goal);
				frontier.put(next, priority);
				came_from[next] = current;
			}
		}
	}
}

std::vector<GridLocation> reconstruct_path(
		GridLocation start, GridLocation goal,
		std::map<GridLocation, GridLocation> came_from) {
	std::vector<GridLocation> path;
	GridLocation current = goal;
	while (current != start) {
		path.push_back(current);
		current = came_from[current];
	}
	path.push_back(start); // optional
	reverse(path.begin(), path.end());
	return path;
}

double poseToYaw(const geometry_msgs::Pose &pose) {
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void publishObstacleMap() {
	if (obstacle_map_publisher.getNumSubscribers() > 0) {
		ros::Time time = ros::Time::now();
		grid_map_msgs::GridMap message;
		obstacle_map.setTimestamp(time.toNSec());
		grid_map::GridMapRosConverter::toMessage(obstacle_map, message);
		obstacle_map_publisher.publish(message);
	}
}

void publishTargetMap() {
	if (target_map_publisher.getNumSubscribers() > 0) {
		ros::Time time = ros::Time::now();
		grid_map_msgs::GridMap message;
		target_map.setTimestamp(time.toNSec());
		grid_map::GridMapRosConverter::toMessage(target_map, message);
		target_map_publisher.publish(message);
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
    const double sonar_depth = 0.1;  // limit view_poly to 10cm past measured ranges
    const double view_range = 2.5;  // don't mark obstacles past this range
	double left = sonarLeft->range;
	double center = sonarCenter->range;
	double right = sonarRight->range;

	// Update the timestamp in the Obstacle map.
	obstacle_map.setTimestamp(ros::Time::now().toNSec());

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
	view_poly.setFrameId(obstacle_map.getFrameId());

	view_poly.addVertex(grid_map::Position(currentLocation.x, currentLocation.y));

	if (sonarLeft->range >= view_range) {
		left = view_range;
	}
	if (sonarCenter->range >= view_range) {
		center = view_range;
	}
	if (sonarRight-> range >= view_range) {
		right = view_range;
	}

	// Left sonar
	view_poly.addVertex(
			grid_map::Position(currentLocation.x + (left + sonar_depth) * cos(currentLocation.theta + M_PI_4),
					currentLocation.y + (left + sonar_depth) * sin(currentLocation.theta + M_PI_4)
			));

	// Center sonar
	view_poly.addVertex(
			grid_map::Position(currentLocation.x + (center + sonar_depth) * cos(currentLocation.theta),
					currentLocation.y + (center + sonar_depth) * sin(currentLocation.theta)
			));

	// Right sonar
	view_poly.addVertex(
			grid_map::Position(currentLocation.x + (right + sonar_depth) * cos(currentLocation.theta - M_PI_4),
					currentLocation.y + (right + sonar_depth) * sin(currentLocation.theta - M_PI_4)
			));

	// Increase the "obstacleness" of the viewable area.
	for (grid_map::PolygonIterator iterator(obstacle_map, view_poly);
	      !iterator.isPastEnd(); ++iterator) {
		double val = obstacle_map.at("obstacles", *iterator);
		if (isnan(val)) {
			val = 0.5;
		}
		val += 0.01;
		if (val > 1)
			val = 1;
	    obstacle_map.at("obstacles", *iterator) = val;
	  }


	grid_map::Polygon clear_poly;
	clear_poly.setFrameId(obstacle_map.getFrameId());

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
	for (grid_map::PolygonIterator iterator(obstacle_map, clear_poly);
	      !iterator.isPastEnd(); ++iterator) {
		double val = obstacle_map.at("obstacles", *iterator);
		if (!isnan(val)) {
			val -= 0.05;
			if (val < 0)
				val = 0;
		}
	    obstacle_map.at("obstacles", *iterator) = val;
	  }

	// Publish the obstacle message if there's an update to it.
	if (next_status != prev_status) {
		swarmie_msgs::Obstacle msg;
		msg.msg = next_status;
		msg.mask = swarmie_msgs::Obstacle::IS_SONAR;
		obstaclePublish.publish(msg);
	}

	prev_status = next_status;

	publishObstacleMap();
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
	target_map.setTimestamp(ros::Time::now().toNSec());

	//
	// Aging. As time goes by, the probability that a block
	// Is still in the place we saw it shrinks. Let's iterate
	// over the map and apply aging.
	//
	grid_map::Matrix &data = target_map["home"];
	grid_map::Matrix &tdata = target_map["target"];
	for (grid_map::GridMapIterator it(target_map); !it.isPastEnd(); ++it) {
		const int i = it.getLinearIndex();
		data(i) *= 0.99;
		tdata(i) *= 0.99;
	}

	if (message->detections.size() > 0) {
		try {
			// The Apriltags package can detect the pose of the tag. The pose
			// in the detections array is relative to the camera. This transform
			// lets us translate the camera coordinates into a world-referenced
			// frame so we can place them on the map.
			//
			// This ensures that the transform is ready to be used below.
			//
			cameraTF->waitForTransform(
					rover + "/odom",   // Target frame
					message->detections[0].pose.header.frame_id, // Source frame
					message->detections[0].pose.header.stamp,    // Time
					ros::Duration(0.1) // How long to wait for the tf.
			);

			for (int i=0; i<message->detections.size(); i++) {

				geometry_msgs::PoseStamped tagpose;
				cameraTF->transformPose(rover + "/odom",
						message->detections[i].pose, tagpose);

				grid_map::Position pos (tagpose.pose.position.x, tagpose.pose.position.y);
				grid_map::Index ind;
				target_map.getIndex(pos, ind);

				if (message->detections[i].id == 0) {
					next_status |= swarmie_msgs::Obstacle::TAG_TARGET;
					target_map.at("target", ind) = 1;
				}
				else if (message->detections[i].id == 256) {
					next_status |= swarmie_msgs::Obstacle::TAG_HOME;
					target_map.at("home", ind) = 1;
					target_map.at("home_yaw", ind) = poseToYaw(tagpose.pose);
				}
			}
		} catch (tf::TransformException &e) {
			ROS_ERROR("%s", e.what());
		}
	}

	if (next_status != prev_status) {
		swarmie_msgs::Obstacle msg;
		msg.msg = next_status;
		msg.mask = swarmie_msgs::Obstacle::IS_VISION;
		obstaclePublish.publish(msg);
	}

	prev_status = next_status;

	publishTargetMap();
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

	// Update the timestamp in both maps.
	uint64_t now = ros::Time::now().toNSec();
	obstacle_map.setTimestamp(now);
	target_map.setTimestamp(now);

//	grid_map::Position newPos(x, y);
//    target_map.move(newPos);

    // Store the current location so we can use it in other places.
    currentLocation.x = x;
    currentLocation.y = y;
    currentLocation.theta = poseToYaw(message->pose.pose);
}

/* Python API
 *
 * find_nearest_target() - Return the position of the nearest tag. Fail
 *   if there are no tags in the map.
 *
 * The service definition is in:
 * 	srv/FindTarget.srv
 *
 */
bool find_neareset_target(mapping::FindTarget::Request &req, mapping::FindTarget::Response &resp) {
	bool rval = false;
	grid_map::Position mypos(currentLocation.x, currentLocation.y);
	for (grid_map::SpiralIterator it(target_map, mypos, 2); !it.isPastEnd(); ++it) {
		if (target_map.at("target", *it) == 0) {
			// Found one!
			resp.result.header.frame_id = target_map.getFrameId();
			resp.result.header.stamp = ros::Time::now();
			grid_map::Position tagpos;
			obstacle_map.getPosition(*it, tagpos);
			resp.result.point.x = tagpos.x();
			resp.result.point.y = tagpos.y();
			resp.result.point.z = 0; // No z() in the map.
			rval = true;
			break;
		}
	}
	return rval;
}

/* Python API
 *
 * get_obstacle_map() - Return a view of the obstacles grid_map.
 *
 * The service definition is in:
 * 	srv/GetMap.srv
 *
 */
bool get_obstacle_map(mapping::GetMap::Request &req, mapping::GetMap::Response &rsp) {
	grid_map::GridMapRosConverter::toMessage(obstacle_map, rsp.map);
	return true;
}

/* Python API
 *
 * get_target_map() - Return a view of the targets grid_map.
 *
 * The service definition is in:
 * 	srv/GetMap.srv
 *
 */
bool get_target_map(mapping::GetMap::Request &req, mapping::GetMap::Response &rsp) {
	grid_map::GridMapRosConverter::toMessage(target_map, rsp.map);
	return true;
}

/*
 * Python API
 *
 * get_plan() - get global plan from a start pose to a goal pose
 * todo: string pulling? use line grid_map iterator?
 * todo: Add 8-connected neighbors. Corner neighbors conditional on adjacents being free
 */
bool get_plan(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &rsp) {
	geometry_msgs::Pose2D start_pose;
	geometry_msgs::Pose2D goal_pose;
    grid_map::Index start_index;
	grid_map::Index goal_index;
	nav_msgs::Path pose_path;

//	start.x = req.start.pose.position.x;
//	start.y = req.start.pose.position.y;
// 	start.theta = poseToYaw(req.start.pose);
//	goal.x = req.goal.pose.position.x;
//	goal.y = req.goal.pose.position.y;
//	goal.theta = poseToYaw(req.goal.pose);

	obstacle_map.getIndex(grid_map::Position(req.start.pose.position.x, req.start.pose.position.y), start_index);
	obstacle_map.getIndex(grid_map::Position(req.goal.pose.position.x, req.goal.pose.position.y), goal_index);

	GridLocation start{start_index(1), start_index(0)};
	GridLocation goal{goal_index(1), goal_index(0)};
	std::map<GridLocation, GridLocation> came_from;
	std::map<GridLocation, double> cost_so_far;

	a_star_search(obstacle_map, start, goal, came_from, cost_so_far);
	std::vector<GridLocation> grid_path = reconstruct_path(start, goal, came_from);

	grid_map::Index index;
	grid_map::Position position;
	std::vector<geometry_msgs::PoseStamped> poses;

	// convert vector of GridLocations to a vector of poses in /odom frame
	for (GridLocation& location : grid_path) {
		index(0) = location.y;
		index(1) = location.x;
		obstacle_map.getPosition(index, position);
		geometry_msgs::PoseStamped pose;
        pose.pose.position.x = position.x();
		pose.pose.position.y = position.y();
		poses.push_back(pose);
	}

	rsp.plan.header.stamp = ros::Time::now();
//	std::vector<geometry_msgs::PoseStamped> poses;
//	poses.push_back(req.goal);
	rsp.plan.poses = poses;
//    rsp.plan.poses.push_back(req.goal);

	pose_path.header.stamp = ros::Time::now();
	pose_path.header.frame_id = rover + "/odom";
	pose_path.poses = poses;
	path_publisher.publish(pose_path);

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

    obstacle_map_publisher = mNH.advertise<grid_map_msgs::GridMap>(rover + "/obstacle_map", 1, false);
    target_map_publisher = mNH.advertise<grid_map_msgs::GridMap>(rover + "/target_map", 1, false);
	path_publisher = mNH.advertise<nav_msgs::Path>(rover + "/plan", 1, false);

    // Services
    //
    // This is the API into the Python control code
    //
    ros::ServiceServer fnt = mNH.advertiseService(rover + "/map/find_nearest_target", find_neareset_target);
    ros::ServiceServer omap = mNH.advertiseService(rover + "/map/get_obstacle_map", get_obstacle_map);
    ros::ServiceServer tmap = mNH.advertiseService(rover + "/map/get_target_map", get_target_map);
	ros::ServiceServer plan = mNH.advertiseService(rover + "/map/get_plan", get_plan);

    // Initialize the maps.
    obstacle_map = grid_map::GridMap({"obstacles"});
    obstacle_map.setFrameId(rover + "/odom");
    obstacle_map.setGeometry(grid_map::Length(25, 25), 0.5);

    target_map = grid_map::GridMap({"target", "home", "home_yaw"});
    target_map.setFrameId(rover + "/odom");
    target_map.setGeometry(grid_map::Length(25, 25), 0.25);

    ros::spin();
    return 0;
}
