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

grid_map::GridMap rover_map;

ros::Publisher obstaclePublish;
ros::Publisher rover_map_publisher;
ros::Publisher path_publisher;

geometry_msgs::Pose2D currentLocation;
tf::TransformListener *cameraTF;

double collisionDistance = 0.6; //meters the ultrasonic detectors will flag obstacles
const double SONAR_ANGLE = 0.436332; // 25 degrees. Mount angles of sonar sensors.
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

// For 4-connected grid
//const std::array<GridLocation, 4> DIRECTIONS =
//		{GridLocation{1, 0}, GridLocation{0, -1}, GridLocation{-1, 0}, GridLocation{0, 1}};

// For 8-connected grid
const std::array<GridLocation, 8> DIRECTIONS =
		{GridLocation{-1, 0}, GridLocation{-1, 1},
		 GridLocation{0, 1}, GridLocation{1, 1},
		 GridLocation{1, 0}, GridLocation{1, -1},
		 GridLocation{0, -1}, GridLocation{-1, -1}};

// Check if location and it's neighbors obstacle values are all below threshold.
bool passable(grid_map::GridMap& map, GridLocation location) {
	const double OBSTACLE_THRESHOLD = 0.25;
    grid_map::Index index;
    index(0) = location.x;
	index(1) = location.y;
	if (map.at("obstacle", index) >= OBSTACLE_THRESHOLD) {
		return false;
	}
	for (GridLocation direction : DIRECTIONS) {
		GridLocation next{location.x + direction.x, location.y + direction.y};
		if (in_bounds(map, next)) {
			index(0) = next.x;
			index(1) = next.y;
			if (map.at("obstacle", index) >= OBSTACLE_THRESHOLD) {
				return false;
			}
		}
	}
	return true;
}

std::vector<GridLocation> neighbors(grid_map::GridMap& map, GridLocation location) {
    std::vector<GridLocation> results;

    for (GridLocation direction : DIRECTIONS) {
        GridLocation next{location.x + direction.x, location.y + direction.y};
        if (in_bounds(map, next) && passable(map, next)) {
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
	// Inflate cost of cells which are neighboring an obstacle
//	const double INFLATION_PCT = 0.7;
	double cost = 1.0; // not yet mapped cells will take this value
	// I think these are the correct indexes for x, y coords
	grid_map::Index index;
	index(0) = to_node.x;
	index(1) = to_node.y;

	if (map.isValid(index, "obstacle")) {
		cost = 1.0 + (map.at("obstacle", index) * 5.0);
	}

//	for (GridLocation neighbor : neighbors(map, to_node)) {
//		index(0) = neighbor.x;
//		index(1) = neighbor.y;
//		if (in_bounds(map, neighbor) && map.isValid(index, "obstacle")) {
//			cost += INFLATION_PCT * (map.at("obstacle", index) + 1) * 100;
//		}
//	}

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

// manhattan distance, for 4-connected grid
//inline double heuristic(GridLocation a, GridLocation b) {
//	return abs(a.x - b.x) + abs(a.y - b.y);
//}

// Octile distance, for 8-connected grid
// http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#diagonal-distance
inline double heuristic(GridLocation a, GridLocation b) {
    const double D = 1;
	const double D2 = 1.41421356237; // sqrt(2)
	double dx = abs(a.x - b.x);
	double dy = abs(a.y - b.y);
	return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);
}

/*
 * A* Search Algorithm
 * Returns true if a path from start to goal is found. Otherwise, returns false.
 */
bool a_star_search(
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
			return true;
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
	return false;
}

/*
/*
 * Rebuild the path from the map of came_from locations.
 * came_from obviously needs to contain a path from goal back to start at this
 * point.
 */
std::vector<GridLocation> reconstruct_path(
		GridLocation start, GridLocation goal,
		std::map<GridLocation, GridLocation> came_from) {
	std::vector<GridLocation> path;
	GridLocation current = goal;
	while (current != start) {
		path.push_back(current);
		current = came_from[current];
	}
    // Don't bother pushing the start location onto the path.
	// path.push_back(start); // optional
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
    // todo: what's a good number for SONAR_DEPTH?
    const double SONAR_DEPTH = 0.5;  // limit mark_poly to 50cm past measured ranges
    // todo: what's a good number for VIEW_RANGE?
	// VIEW_RANGE can help avoid marking "fake" obstacles seen due to sonar
	// noise at longer ranges. It limits the mark_poly to this range, but
	// not clear_poly, so the map can still be cleared past this point.
	// todo: limit clear_poly left and right ranges using VIEW_RANGE?
    const double VIEW_RANGE = 1.5;  // don't mark obstacles past this range

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


	grid_map::Polygon clear_poly;
	clear_poly.setFrameId(rover_map.getFrameId());

	clear_poly.addVertex(grid_map::Position(currentLocation.x, currentLocation.y));

	// Left sonar
	clear_poly.addVertex(
			grid_map::Position(currentLocation.x + sonarLeft->range * cos(currentLocation.theta + SONAR_ANGLE),
					currentLocation.y + sonarLeft->range * sin(currentLocation.theta + SONAR_ANGLE)
			));

	// Center sonar
	clear_poly.addVertex(
			grid_map::Position(currentLocation.x + sonarCenter->range * cos(currentLocation.theta),
					currentLocation.y + sonarCenter->range * sin(currentLocation.theta)
			));

	// Right sonar
	clear_poly.addVertex(
			grid_map::Position(currentLocation.x + sonarRight->range * cos(currentLocation.theta - SONAR_ANGLE),
					currentLocation.y + sonarRight->range * sin(currentLocation.theta - SONAR_ANGLE)
			));

	// Clear the area that's clear in sonar.
	for (grid_map::PolygonIterator iterator(rover_map, clear_poly);
	      !iterator.isPastEnd(); ++iterator) {
		double val = rover_map.at("obstacle", *iterator);
		if (isnan(val)) {
			val = 0;
		}
        val -= 0.05;
        if (val < 0)
            val = 0;
	    rover_map.at("obstacle", *iterator) = val;
	  }

	grid_map::Polygon mark_poly;
	mark_poly.setFrameId(rover_map.getFrameId());
	bool do_marking = false;

	if (sonarLeft->range < VIEW_RANGE) {
		do_marking = true;
		// Left sonar
		mark_poly.addVertex(
				grid_map::Position(currentLocation.x + sonarLeft->range * cos(currentLocation.theta + SONAR_ANGLE),
					currentLocation.y + sonarLeft->range * sin(currentLocation.theta + SONAR_ANGLE)
				));
		mark_poly.addVertex(
				grid_map::Position(currentLocation.x + (sonarLeft->range + SONAR_DEPTH) * cos(currentLocation.theta + SONAR_ANGLE),
                    currentLocation.y + (sonarLeft->range + SONAR_DEPTH) * sin(currentLocation.theta + SONAR_ANGLE)
				));
	}

	if (sonarCenter->range < VIEW_RANGE) {
		do_marking = true;
		// Center sonar
		mark_poly.addVertex(
				grid_map::Position(currentLocation.x + sonarCenter->range * cos(currentLocation.theta),
					currentLocation.y + sonarCenter->range * sin(currentLocation.theta)
				));
		mark_poly.addVertex(
				grid_map::Position(currentLocation.x + (sonarCenter->range + SONAR_DEPTH) * cos(currentLocation.theta),
                    currentLocation.y + (sonarCenter->range + SONAR_DEPTH) * sin(currentLocation.theta)
				));
	}

	if (sonarRight->range < VIEW_RANGE) {
        do_marking = true;
		// Right sonar
		mark_poly.addVertex(
				grid_map::Position(currentLocation.x + sonarRight->range * cos(currentLocation.theta - SONAR_ANGLE),
                    currentLocation.y + sonarRight->range * sin(currentLocation.theta - SONAR_ANGLE)
				));
		mark_poly.addVertex(
				grid_map::Position(currentLocation.x + (sonarRight->range + SONAR_DEPTH) * cos(currentLocation.theta - SONAR_ANGLE),
                    currentLocation.y + (sonarRight->range + SONAR_DEPTH) * sin(currentLocation.theta - SONAR_ANGLE)
				));
	}

	if (do_marking) {
		// Mark the area that's blocked in sonar.
		double avg_range = (sonarLeft->range + sonarCenter->range + sonarRight->range) / 3.0;
		for (grid_map::PolygonIterator iterator(rover_map, mark_poly);
			 !iterator.isPastEnd(); ++iterator) {
			double val = rover_map.at("obstacle", *iterator);
			if (isnan(val)) {
				val = 0.1;
			}
			val += 0.01 * (3.0 / avg_range); // mark nearer obstacles faster
			if (val > 1)
				val = 1;
			rover_map.at("obstacle", *iterator) = val;
		  }
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

	/*
	 * Clear the polygon that the camera sees. This is necessary to erase
	 * grid cells where blocks have disappeared.
	 */
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


	grid_map::Matrix& home_layer = rover_map["home"];
	grid_map::Matrix& target_layer = rover_map["target"];

	for (grid_map::PolygonIterator iterator(rover_map, view_poly);
	      !iterator.isPastEnd(); ++iterator) {
	    const grid_map::Index index(*iterator);
	    home_layer(index(0), index(1)) = 0;
	    target_layer(index(0), index(1)) = 0;
	}

	// Handle target detections

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
				rover_map.getIndex(pos, ind);

				if (message->detections[i].id == 0) {
					next_status |= swarmie_msgs::Obstacle::TAG_TARGET;
					rover_map.at("target", ind) = 1;
				}
				else if (message->detections[i].id == 256) {
					next_status |= swarmie_msgs::Obstacle::TAG_HOME;
					rover_map.at("home", ind) = 1;
				}
			}
		} catch (tf::TransformException &e) {
			ROS_ERROR("%s", e.what());
		}

		for (int i=0; i<message->detections.size(); i++) {
			if (message->detections[i].id == 0) {
				next_status |= swarmie_msgs::Obstacle::TAG_TARGET;
			}
			else if (message->detections[i].id == 256) {
				next_status |= swarmie_msgs::Obstacle::TAG_HOME;
			}
		}
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

/*
 * Python API
 *
 * get_plan() - get global plan from a start pose to a goal pose
 * todo: string pulling? use line grid_map iterator?
 * todo: Confirm on physical rover that 8-connected passable() neighbors check is fast enough
 * todo: include apriltag layers in path plan
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

	rover_map.getIndex(grid_map::Position(req.start.pose.position.x, req.start.pose.position.y), start_index);
	rover_map.getIndex(grid_map::Position(req.goal.pose.position.x, req.goal.pose.position.y), goal_index);

	GridLocation start{start_index(0), start_index(1)};
	GridLocation goal{goal_index(0), goal_index(1)};
	std::map<GridLocation, GridLocation> came_from;
	std::map<GridLocation, double> cost_so_far;

	bool success = a_star_search(rover_map, start,
								 goal, came_from, cost_so_far);
	if (!success) {
		return false;
	}
	std::vector<GridLocation> grid_path = reconstruct_path(start, goal, came_from);

	grid_map::Index index;
	grid_map::Position position;
	std::vector<geometry_msgs::PoseStamped> poses;

	// convert vector of GridLocations to a vector of poses in /odom frame
	for (GridLocation& location : grid_path) {
		index(0) = location.x;
		index(1) = location.y;
		rover_map.getPosition(index, position);
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

    rover_map_publisher = mNH.advertise<grid_map_msgs::GridMap>(rover + "/map", 1, false);
	path_publisher = mNH.advertise<nav_msgs::Path>(rover + "/plan", 1, false);

    // Services
    //
    // This is the API into the Python control code
    //
    ros::ServiceServer omap = mNH.advertiseService(rover + "/map/get_map", get_map);
	ros::ServiceServer plan = mNH.advertiseService(rover + "/map/get_plan", get_plan);

    // Initialize the maps.
    rover_map = grid_map::GridMap({"obstacle", "target", "home"});
    rover_map.setFrameId(rover + "/odom");
    rover_map.setGeometry(grid_map::Length(25, 25), 0.5);

    ros::spin();
    return 0;
}
