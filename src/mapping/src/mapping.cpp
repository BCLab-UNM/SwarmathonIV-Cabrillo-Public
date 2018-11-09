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
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>

#include <swarmie_msgs/Obstacle.h>
#include <mapping/FindTarget.h>
#include <mapping/GetMap.h>
#include <mapping/GetNavPlan.h>

#include <execinfo.h>
#include <signal.h>

using namespace std;

grid_map::GridMap rover_map;

ros::Publisher obstaclePublish;
ros::Publisher rover_map_publisher;
ros::Publisher path_publisher;

geometry_msgs::Pose2D currentLocation;
bool isMoving = false;
tf::TransformListener *cameraTF;

double singleSensorCollisionDist = 0.75; // meters a single sensor will flag an obstacle
double doubleSensorCollisionDist = 1.2; //meters the two sensors will flag obstacles
const double SONAR_ANGLE = 0.436332; // 25 degrees. Mount angles of sonar sensors.
const double MAP_RESOLUTION = 0.5; // map resolution, meters per cell
unsigned int obstacle_status;

std::string map_frame;

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

// Neighbors for 8-connected grid
const std::array<GridLocation, 8> DIRECTIONS =
		{GridLocation{-1, 0}, GridLocation{-1, 1},
		 GridLocation{0, 1}, GridLocation{1, 1},
		 GridLocation{1, 0}, GridLocation{1, -1},
		 GridLocation{0, -1}, GridLocation{-1, -1}};

// 2-step away neighbors for an 8-connected grid
const std::array<GridLocation, 16> TWO_STEP_DIRECTIONS =
		{GridLocation{-2, 0}, GridLocation{-2, 1}, GridLocation{-2, 2},
		 GridLocation{-1, 2}, GridLocation{0, 2}, GridLocation{1, 2},
		 GridLocation{2, 2}, GridLocation{2, 1}, GridLocation{2, 0},
		 GridLocation{2, -1}, GridLocation{2, -2}, GridLocation{1, -2},
		 GridLocation{0, -2}, GridLocation{-1, -2}, GridLocation{-2, -2},
		 GridLocation{-2, -1}};

/*
 * Check if location and it's neighbors obstacle values are all below threshold.
 * location_from and location_to must be in_bounds
 */
bool passable(grid_map::GridMap& map, GridLocation location_from,
			  GridLocation location_to, bool use_home_layer) {
	const double OBSTACLE_THRESHOLD = 0.10;
    grid_map::Index index;
    index(0) = location_to.x;
	index(1) = location_to.y;
    GridLocation direction{location_to.x - location_from.x,
						   location_to.y - location_from.y};
	if (map.at("obstacle", index) >= OBSTACLE_THRESHOLD) {
		return false;
	}
    if (use_home_layer && map.at("home", index) >= OBSTACLE_THRESHOLD) {
		return false;
	}

	if (direction.x != 0 || direction.y != 0) { // it's a diagonal direction
		grid_map::Index adjacent_x_axis(direction.x, direction.y & 0);
		grid_map::Index adjacent_y_axis(direction.x & 0, direction.y);

		// not passable if both adjacent neighbors of diagonal are blocked
		if (use_home_layer) {
			if ((map.at("obstacle", adjacent_x_axis) >= OBSTACLE_THRESHOLD ||
				 map.at("home", adjacent_x_axis) >= OBSTACLE_THRESHOLD)
				&&
				(map.at("obstacle", adjacent_y_axis) >= OBSTACLE_THRESHOLD ||
				 map.at("home", adjacent_y_axis) >= OBSTACLE_THRESHOLD)) {
				return false;
			}
		} else if (map.at("obstacle", adjacent_x_axis) >= OBSTACLE_THRESHOLD ||
				   map.at("obstacle", adjacent_y_axis) >= OBSTACLE_THRESHOLD) {
			return false;
		}
	}

	return true;
}

std::vector<GridLocation> neighbors(grid_map::GridMap& map,
									GridLocation location,
									bool use_home_layer) {
    std::vector<GridLocation> results;

    for (GridLocation direction : DIRECTIONS) {
        GridLocation next{location.x + direction.x, location.y + direction.y};
        if (in_bounds(map, next) &&
				passable(map, location, next, use_home_layer)) {
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
double cost(grid_map::GridMap& map,
			GridLocation to_node, bool use_home_layer) {
	// Inflate cost of cells which are neighboring an obstacle
	const double INFLATION_PCT = 0.7;
	const double LETHAL_COST = 255;
	const double NEUTRAL_COST = 50; // not yet mapped cells will take this value
	// minimum cost for a cell, must match the heuristic. This is slightly
	// larger than sqrt(2), the heuristic value for a diagonal move
	double cost = 1.5;

	grid_map::Index index;
	index(0) = to_node.x;
	index(1) = to_node.y;

	if (map.isValid(index, "obstacle")) {
        cost += 2 * LETHAL_COST * map.at("obstacle", index);
	}
	if (use_home_layer && map.isValid(index, "home")) {
		cost += LETHAL_COST * map.at("home", index);
	}

	if (cost > LETHAL_COST) { // skip 2-layer neighbors check if possible
		cost = LETHAL_COST;
		return cost;
	}

    for (GridLocation direction : DIRECTIONS) {
        GridLocation next{to_node.x + direction.x, to_node.y + to_node.y};
		index(0) = next.x;
		index(1) = next.y;
        if (in_bounds(map, next)) {
            if (map.isValid(index, "obstacle")) {
                cost += INFLATION_PCT * 2 * LETHAL_COST *
                        map.at("obstacle", index);
            }
            if (use_home_layer && map.isValid(index, "home")) {
                cost += INFLATION_PCT * LETHAL_COST *
                        map.at("home", index);
            }
        }
    }

    for (GridLocation direction : TWO_STEP_DIRECTIONS) {
        GridLocation next{to_node.x + direction.x, to_node.y + to_node.y};
		index(0) = next.x;
		index(1) = next.y;
        if (in_bounds(map, next)) {
            if (map.isValid(index, "obstacle")) {
                cost += INFLATION_PCT / 2.0 * 2 * LETHAL_COST *
                        map.at("obstacle", index);
            }
            if (use_home_layer && map.isValid(index, "home")) {
                cost += INFLATION_PCT / 2.0 * LETHAL_COST *
                        map.at("home", index);
            }
        }
    }

	if (cost > LETHAL_COST) {
		cost = LETHAL_COST;
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



/*
 * Helper to a_star_search() and at_goal()
 *
 * The hueristic for A* approximates the minimum distance between location
 * a and location b, if the route between them was free of any obstacles.
 *
 * Octile distance, for 8-connected grid
 * http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#diagonal-distance
 */
inline double heuristic(GridLocation a, GridLocation b) {
    const double D = 1;
	const double D2 = 1.41421356237; // sqrt(2)
	double dx = abs(a.x - b.x);
	double dy = abs(a.y - b.y);
	return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);
}

/*
 * Helper to a_star_search()
 *
 * Returns true if distance from current location to goal location is
 * within the acceptable tolerance.
 */
inline bool at_goal(GridLocation current, GridLocation goal, int tolerance) {
	return heuristic(current, goal) <= tolerance;
}

/*
 * A* Search Algorithm
 *
 * Returns true if a path from start to goal is found.
 * Otherwise, returns false.
 *
 * Looks for the shortest path in map from start to goal. Stops when current
 * cell is within tolerance distance of goal cell.
 *
 * If using tolerance > 0 and the search completes successfully, the goal
 * location will be modified here to be the location where the search ended.
 * This way, reconstruct path will still function normally.
 */
bool a_star_search(
		grid_map::GridMap& map,
        GridLocation start,
        GridLocation& goal,
		int tolerance,
		bool use_home_layer,
        std::map<GridLocation, GridLocation>& came_from,
        std::map<GridLocation, double>& cost_so_far) {

	PriorityQueue<GridLocation, double> frontier;
	frontier.put(start, 0);

	came_from[start] = start;
	cost_so_far[start] = 0;

	while (!frontier.empty()) {
		GridLocation current = frontier.get();

		if (at_goal(current, goal, tolerance)) {
			goal = current; // modify goal location so path rebuilding works
			return true;
		}

		for (GridLocation next : neighbors(map, current, use_home_layer)) {
			double new_cost = cost_so_far[current]
							  + cost(map, next, use_home_layer);

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
 * Helper to straighten_path(). Returns true if end is in line of sight of
 * start. Returns false otherwise.
 *
 * end is in line of sight if line iterator between the two indexes doesn't
 * cross a cell with value above OBSTACLE_THRESHOLD
 */
bool in_line_of_sight(
		grid_map::GridMap& map,
		grid_map::Index start,
		grid_map::Index end,
		bool use_home_layer) {
	const double OBSTACLE_THRESHOLD = 0.10;
	for (grid_map::LineIterator iterator(map, start, end);
		 !iterator.isPastEnd(); ++iterator) {
        if (map.isValid(*iterator, "obstacle")) {
            if (map.at("obstacle", *iterator) > OBSTACLE_THRESHOLD) {
				return false;
			}
			if (use_home_layer &&
					map.at("home", *iterator) > OBSTACLE_THRESHOLD) {
                return false;
            }
        }
	}
	return true;
}

/*
 * Straighten first part of the path by finding the furthest waypoint that's
 * still in line-of-sight (not blocked by obstacles). Straightened path will
 * begin with the furthest waypoint in line-of-sight, followed by the remaining
 * original waypoints.
 *
 * Returns a vector containing the new, straightened path.
 * Returns the original path if path has 1 or fewer waypoints.
 */
std::vector<GridLocation> straighten_path(grid_map::GridMap& map,
										  std::vector<GridLocation> path,
										  bool use_home_layer) {
	std::vector<GridLocation> result;

	if (path.size() <= 1) {
		return path;
	}

	grid_map::Index start(path[0].x, path[0].y);
	unsigned int i = 1;
    while (i < path.size()) {
		grid_map::Index end(path[i].x, path[i].y);
        if (!in_line_of_sight(map, start, end, use_home_layer)) {
			break;
		}
		i++;
	}

    // Only include this point if it isn't the start point
    if (i > 1) {
		result.push_back(path[i - 1]); // last in line-of-sight location
    }
	while (i < path.size()) {
		result.push_back(path[i]);
		i++;
	}

	return result;
}

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
	// todo: put this back now that path straightening is in?
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

void publishRoverMap() {
	if (rover_map_publisher.getNumSubscribers() > 0) {
		ros::Time time = ros::Time::now();
		grid_map_msgs::GridMap message;
		rover_map.setTimestamp(time.toNSec());
		grid_map::GridMapRosConverter::toMessage(rover_map, message);
		rover_map_publisher.publish(message);
	}
}

/*
 * Helper to sonarHandler() and targetHandler()
 * Returns val decreased by rate, with minimum val zero.
 * Returns zero if val is not a number (isnan(val)) or if decreasing val by
 * rate would reduce val to < 0.
 */
double decreaseVal(double val, double rate) {
	if (isnan(val))
		val = 0;
	val -= rate;
	if (val < 0)
		val = 0;
	return val;
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

	// Minimum distance sonar center obstacles will be marked at. Anything
	// inside this distance should be a block in the claw.
    const double MIN_CENTER_DIST = 0.15;
    // todo: what's a good number for SONAR_DEPTH?
    const double SONAR_DEPTH = 0.75;  // limit mark_poly to 75cm past measured ranges
    // todo: what's a good number for VIEW_RANGE?
	// VIEW_RANGE can help avoid marking "fake" obstacles seen due to sonar
	// noise at longer ranges. It limits the mark_poly to this range, but
	// not clear_poly, so the map can still be cleared past this point.
	// todo: limit clear_poly left and right ranges using VIEW_RANGE?
    const double VIEW_RANGE = 1.0;  // don't mark obstacles past this range

	// Update the timestamp in the Obstacle map.
	rover_map.setTimestamp(ros::Time::now().toNSec());

	// Calculate the obstacle status.
	// Single sensor ranges below single sensor threshold can trigger an
	// obstacle message.
	if (sonarLeft->range < singleSensorCollisionDist) {
		next_status |= swarmie_msgs::Obstacle::SONAR_LEFT;
	}
    if (sonarCenter->range < singleSensorCollisionDist &&
        sonarCenter->range > 0.12) {
		next_status |= swarmie_msgs::Obstacle::SONAR_CENTER;
	}
    if (sonarRight->range < singleSensorCollisionDist) {
		next_status |= swarmie_msgs::Obstacle::SONAR_RIGHT;
	}

	// Two adjacent sensors both below double sensor threshold can also trigger
	// an obstacle message.
	if (sonarLeft->range < doubleSensorCollisionDist && sonarCenter->range < doubleSensorCollisionDist) {
		next_status |= swarmie_msgs::Obstacle::SONAR_LEFT;
		next_status |= swarmie_msgs::Obstacle::SONAR_CENTER;
	}
	if (sonarRight->range < doubleSensorCollisionDist && sonarCenter->range < doubleSensorCollisionDist) {
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
	// Only use if its range is far enough away to definitely not be a block
	// in the claw.
	if (sonarCenter->range > MIN_CENTER_DIST) {
		clear_poly.addVertex(
				grid_map::Position(currentLocation.x + sonarCenter->range *
				cos(currentLocation.theta),
						currentLocation.y + sonarCenter->range * sin(currentLocation.theta)
				));
	}

	// Right sonar
	clear_poly.addVertex(
			grid_map::Position(currentLocation.x + sonarRight->range * cos(currentLocation.theta - SONAR_ANGLE),
					currentLocation.y + sonarRight->range * sin(currentLocation.theta - SONAR_ANGLE)
			));

	// Clear the area that's clear in sonar.
	for (grid_map::PolygonIterator iterator(rover_map, clear_poly);
	      !iterator.isPastEnd(); ++iterator) {
		double val = rover_map.at("obstacle", *iterator);
	    rover_map.at("obstacle", *iterator) = decreaseVal(val, 0.05);
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

	if (sonarCenter->range > MIN_CENTER_DIST && sonarCenter->range < VIEW_RANGE) {
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

/* targetHandler() - Called when there's new Apriltag detection data.
 *
 * This does two things:
 *
 * 1. Send and obstacle message if there's a visible tag. TAG_TARGET's are only
 *    considered obstacles if they are far enough from the camera to definitely
 *    not be a block in the claw.
 *
 * 2. Update the target maps based on current detections.
 *
 */
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
	// Measurements defining camera field of view for polygon iterator
	const double CAMERA_NEAR_ANGLE = 0.28; // radians
	const double CAMERA_FAR_ANGLE = 0.34;
	const double CAMERA_NEAR_DIST = 0.29; // meters
	const double CAMERA_FAR_DIST = 0.74;

	// TAG_TARGET detections closer than this won't be marked as obstacles
	const double TAG_IN_CLAW_DIST = 0.22; // meters

	// Clear camera field of view at different rates if moving or stopped
	// todo: are these rates good?
	const double MOVING_CLEAR_RATE = 0.03;
	const double STOPPED_CLEAR_RATE = 0.3;

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

	// Near left corner
	view_poly.addVertex(
			grid_map::Position(currentLocation.x + CAMERA_NEAR_DIST * cos(currentLocation.theta + CAMERA_NEAR_ANGLE),
					currentLocation.y + CAMERA_NEAR_DIST * sin(currentLocation.theta + CAMERA_NEAR_ANGLE)
			));

	// Far left corner
	view_poly.addVertex(
			grid_map::Position(currentLocation.x + CAMERA_FAR_DIST * cos(currentLocation.theta + CAMERA_FAR_ANGLE),
					currentLocation.y + CAMERA_FAR_DIST * sin(currentLocation.theta + CAMERA_FAR_ANGLE)
			));

	// Far right corner
    view_poly.addVertex(
			grid_map::Position(currentLocation.x + CAMERA_FAR_DIST * cos(currentLocation.theta - CAMERA_FAR_ANGLE),
					currentLocation.y + CAMERA_FAR_DIST * sin(currentLocation.theta - CAMERA_FAR_ANGLE)
			));

	// Near right corner
    view_poly.addVertex(
			grid_map::Position(currentLocation.x + CAMERA_NEAR_DIST * cos(currentLocation.theta - CAMERA_NEAR_ANGLE),
					currentLocation.y + CAMERA_NEAR_DIST * sin(currentLocation.theta - CAMERA_NEAR_ANGLE)
			));

	double rate = STOPPED_CLEAR_RATE;
	if (isMoving) {
		rate = MOVING_CLEAR_RATE;
	}

	grid_map::Matrix& home_layer = rover_map["home"];
	grid_map::Matrix& target_layer = rover_map["target"];

	for (grid_map::PolygonIterator iterator(rover_map, view_poly);
	      !iterator.isPastEnd(); ++iterator) {
	    const grid_map::Index index(*iterator);
		double val = home_layer(index(0), index(1));
        home_layer(index(0), index(1)) = decreaseVal(val, rate);

        val = target_layer(index(0), index(1));
		target_layer(index(0), index(1)) = decreaseVal(val, rate);
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
			// Wait for 0.2 seconds to try to avoid transform exceptions
			//
			cameraTF->waitForTransform(
					map_frame,   // Target frame
					message->detections[0].pose.header.frame_id, // Source frame
					message->detections[0].pose.header.stamp,    // Time
					ros::Duration(0.2) // How long to wait for the tf.
			);

            for (int i=0; i<message->detections.size(); i++) {
                geometry_msgs::PoseStamped tagpose;
                cameraTF->transformPose(map_frame,
                                        message->detections[i].pose,
                                        tagpose);

                grid_map::Position pos(tagpose.pose.position.x,
                                       tagpose.pose.position.y);
                grid_map::Index ind;
                rover_map.getIndex(pos, ind);

                // Only consider TAG_TARGET's far enough away from camera
                // to avoid marking block in claw as an obstacle.
                if (message->detections[i].id == 0 &&
                    message->detections[i].pose.pose.position.z > TAG_IN_CLAW_DIST) {
                    rover_map.at("target", ind) = 1;
                } else if (message->detections[i].id == 256) {
                    rover_map.at("home", ind) = 1;
                }
			}
		} catch (tf::TransformException &e) {
			ROS_ERROR("%s", e.what());
		}

        // Make sure Obstacle messages get published, so do this here, outside
		// the try/catch block for transforms
		for (int i=0; i<message->detections.size(); i++) {
			if (message->detections[i].id == 0 &&
				message->detections[i].pose.pose.position.z > TAG_IN_CLAW_DIST) {
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
 * Store current 2D Pose and see if we're moving.
 */
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
    double x = message->pose.pose.position.x;
    double y = message->pose.pose.position.y;

    // Store the current location so we can use it in other places.
    currentLocation.x = x;
    currentLocation.y = y;
    currentLocation.theta = poseToYaw(message->pose.pose);

	isMoving = (abs(message->twist.twist.linear.x) > 0.1
				|| abs(message->twist.twist.angular.z > 0.2));
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
 * todo: Confirm on physical rover that 8-connected passable() neighbors check is fast enough
 */
bool get_plan(mapping::GetNavPlan::Request &req,
			  mapping::GetNavPlan::Response &rsp) {
    grid_map::Index start_index;
	grid_map::Index goal_index;
	nav_msgs::Path pose_path;

	// whether to use "home" layer in path search
	bool use_home_layer = req.use_home_layer.data;

	// Return service error (false) if start or goal outside map boundaries.
	if (!rover_map.getIndex(
			grid_map::Position(req.start.pose.position.x,
							   req.start.pose.position.y),
			start_index
	)) {
		return false;
	}
	if (!rover_map.getIndex(
			grid_map::Position(req.goal.pose.position.x,
							   req.goal.pose.position.y),
			goal_index
	)) {
		return false;
	}

	GridLocation start{start_index(0), start_index(1)};
	GridLocation goal{goal_index(0), goal_index(1)};

	int tolerance = 0; // default tolerance for search, must reach goal exactly
	if (req.tolerance > 0) {
		tolerance = int(req.tolerance / MAP_RESOLUTION);
	}

	std::map<GridLocation, GridLocation> came_from;
	std::map<GridLocation, double> cost_so_far;

	bool success = a_star_search(rover_map, start, goal, tolerance,
								 use_home_layer, came_from, cost_so_far);
	if (!success) {
		return false;
	}
	std::vector<GridLocation> grid_path = straighten_path(
			rover_map,
			reconstruct_path(start, goal, came_from),
			use_home_layer
	);

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
	pose_path.header.frame_id = map_frame;
	pose_path.poses = poses;
	path_publisher.publish(pose_path);

	return true;
}

void crashHandler(int s) {
  int j, nptrs;
  void *buffer[1000];

  nptrs = backtrace(buffer, 1000);
  printf("backtrace() returned %d addresses\n", nptrs);
  backtrace_symbols_fd(buffer, nptrs, STDOUT_FILENO);
  exit(-s);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mapping");
    ros::NodeHandle mNH;

    signal(SIGSEGV, crashHandler);

    obstacle_status = swarmie_msgs::Obstacle::PATH_IS_CLEAR;

    // Parameters
    ros::param::param<std::string>("odom_frame", map_frame, "odom");
    
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
    ros::Subscriber odomSubscriber = mNH.subscribe("odom/filtered", 10, odometryHandler);
    ros::Subscriber targetSubscriber = mNH.subscribe("targets", 10, targetHandler);

    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, "sonarLeft", 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, "sonarCenter", 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, "sonarRight", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    //	Publishers

    obstaclePublish = mNH.advertise<swarmie_msgs::Obstacle>("obstacle", 1, true);

    rover_map_publisher = mNH.advertise<grid_map_msgs::GridMap>("map", 1, false);
	path_publisher = mNH.advertise<nav_msgs::Path>("plan", 1, false);

    // Services
    //
    // This is the API into the Python control code
    //
    ros::ServiceServer omap = mNH.advertiseService("map/get_map", get_map);
	ros::ServiceServer plan = mNH.advertiseService("map/get_plan", get_plan);

    // Initialize the maps.
    rover_map = grid_map::GridMap({"obstacle", "target", "home"});
    rover_map.setFrameId(map_frame);
    rover_map.setGeometry(grid_map::Length(25, 25), MAP_RESOLUTION);

    ros::spin();
    return 0;
}
