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

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/client.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <apriltags2to1/AprilTagDetectionArray.h>
#include <apriltags2to1/AprilTagDetection.h>
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
#include <mapping/mappingConfig.h>

#include <execinfo.h>
#include <signal.h>

using namespace std;

grid_map::GridMap rover_map;

ros::Publisher obstaclePublish;
ros::Publisher rover_map_publisher;
ros::Publisher path_publisher;

geometry_msgs::Pose2D currentLocation;
bool isMoving = false;
tf::TransformListener *tf_l;

// Dynamic reconfigure params
bool params_configured = false; // wait until the parameters are initialized

// param group map
double sonar_fov;  // Field of view of the sonar sensors (rad).
double cos_fov_2;  // store the cos(sonar_fov / 2) for repeated use.
double sin_fov_2;  // store the sin(sonar_fov / 2) for repeated use.
double single_sensor_obst_dist; // meters a single sensor will flag an obstacle
double double_sensor_obst_dist; //meters the two sensors will flag obstacles
// todo: what's a good number for sonar_view_range?
// sonar_view_range can help avoid marking "fake" obstacles seen due to sonar
// noise at longer ranges. It limits the mark_poly to this range, but
// not clear_poly, so the map can still be cleared past this point.
// todo: limit clear_poly left and right ranges using sonar_view_range?
double sonar_view_range;  // don't mark obstacles past this range
// todo: what's a good number for sonar_obst_depth?
double sonar_obst_depth; // limit mark_poly to this dist past measured ranges
double sonar_base_mark_rate;
double sonar_base_clear_rate;

// param group search
double obstacle_threshold; // min value for a cell to be considered impassable
double inflation_pct; // Inflate cost of cells which are neighboring an obstacle
double lethal_cost;
// Not yet mapped obstacle layer cells will take this neutral value. This
// helps to make the search prefer previously visited areas.
double neutral_cost;
bool visualize_frontier;

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


/*
 * Check if location and it's neighbors obstacle values are all below threshold.
 * location_from and location_to must be in_bounds
 */
bool passable(grid_map::GridMap& map, GridLocation location_from,
              GridLocation location_to, bool use_home_layer) {
    // TODO: make passable() only use location_to's value, i.e. diagonal adjacents? In theory the cost() neighbor inflation should cover this case
    grid_map::Index index;
    index(0) = location_to.x;
    index(1) = location_to.y;
    GridLocation direction{location_to.x - location_from.x,
                           location_to.y - location_from.y};
    if (map.at("obstacle", index) >= obstacle_threshold) {
        return false;
    }
    if (use_home_layer && map.at("home", index) >= obstacle_threshold) {
        return false;
    }

    if (direction.x != 0 || direction.y != 0) { // it's a diagonal direction
        grid_map::Index adjacent_x_axis(direction.x, direction.y & 0);
        grid_map::Index adjacent_y_axis(direction.x & 0, direction.y);

        // not passable if both adjacent neighbors of diagonal are blocked
        if (use_home_layer) {
            if ((map.at("obstacle", adjacent_x_axis) >= obstacle_threshold ||
                 map.at("home", adjacent_x_axis) >= obstacle_threshold)
                &&
                (map.at("obstacle", adjacent_y_axis) >= obstacle_threshold ||
                 map.at("home", adjacent_y_axis) >= obstacle_threshold)) {
                return false;
            }
        } else if (map.at("obstacle", adjacent_x_axis) >= obstacle_threshold ||
                   map.at("obstacle", adjacent_y_axis) >= obstacle_threshold) {
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
    // The minimum cost for a cell, must match the heuristic. This is slightly
    // larger than sqrt(2), the heuristic value for a diagonal move
    double cost = 1.5;

    grid_map::Index index;
    index(0) = to_node.x;
    index(1) = to_node.y;

    if (map.isValid(index, "obstacle")) {
        cost += lethal_cost * map.at("obstacle", index);
    } else {
        cost = neutral_cost;
    }
    if (use_home_layer && map.isValid(index, "home")) {
        cost += lethal_cost * map.at("home", index);
    }

    if (cost > lethal_cost) { // skip 2-layer neighbors check if possible
        cost = lethal_cost;
        return cost;
    }

    for (GridLocation direction : DIRECTIONS) {
        GridLocation next{to_node.x + direction.x, to_node.y + direction.y};
        index(0) = next.x;
        index(1) = next.y;
        if (in_bounds(map, next)) {
            if (map.isValid(index, "obstacle")) {
                cost += inflation_pct * lethal_cost *
                        map.at("obstacle", index);
            }
            if (use_home_layer && map.isValid(index, "home")) {
                cost += inflation_pct * lethal_cost *
                        map.at("home", index);
            }
        }
    }


    if (cost > lethal_cost) {
        cost = lethal_cost;
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
 * cross a cell with value above obstacle_threshold
 */
bool in_line_of_sight(
    grid_map::GridMap& map,
    grid_map::Index start,
    grid_map::Index end,
    bool use_home_layer) {

    for (grid_map::LineIterator iterator(map, start, end);
         !iterator.isPastEnd(); ++iterator) {
        if (map.isValid(*iterator, "obstacle")) {
            if (map.at("obstacle", *iterator) > obstacle_threshold) {
                return false;
            }
            if (use_home_layer &&
                    map.at("home", *iterator) > obstacle_threshold) {
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
 * Helper to clearSonar() and targetHandler()
 * Returns val decreased by rate, with minimum val zero.
 * Returns zero if val is not a number (isnan(val)) or if decreasing val by
 * rate would reduce val to < 0.
 */
inline double decreaseVal(double val, double rate) {
    if (isnan(val))
        val = 0;
    val -= rate;
    if (val < 0)
        val = 0;
    return val;
}

/*
 * Helper to markSonar()
 * Returns val increased by rate, with maximum val one.
 * Returns rate if val is not a number (isnan(val)).
 */
inline double increaseVal(double val, double rate) {
    if (isnan(val))
        val = rate;
    val += rate;
    if (val > 1)
        val = 1;
    return val;
}

/*
 * Helper to clearSonar() and markSonar()
 * fov_pts will contain the three points in the sonar coordinate frame, to be
 * transformed and then used in the appropriate grid map polygon iterator.
 */
void polygonFovPts(
    const sensor_msgs::Range::ConstPtr& sonar,
    std::vector<geometry_msgs::PointStamped>& fov_pts,
    bool add_depth=false) {

    geometry_msgs::PointStamped fov_us_l;
    fov_us_l.header = sonar->header;
    fov_us_l.point.x = cos_fov_2 * sonar->range;
    fov_us_l.point.y = sin_fov_2 * sonar-> range;
    fov_pts.push_back(fov_us_l);

    geometry_msgs::PointStamped fov_us_c;
    fov_us_c.header = sonar->header;
    fov_us_c.point.x = sonar->range;
    fov_pts.push_back(fov_us_c);

    geometry_msgs::PointStamped fov_us_r;
    fov_us_r.header = sonar->header;
    fov_us_r.point.x = fov_us_l.point.x;
    fov_us_r.point.y = -fov_us_l.point.y;
    fov_pts.push_back(fov_us_r);

    if (add_depth) {
        geometry_msgs::PointStamped fov_us_l_depth;
        fov_us_l_depth.header = sonar->header;
        fov_us_l_depth.point.x = cos_fov_2 * (sonar->range + sonar_obst_depth);
        fov_us_l_depth.point.y = sin_fov_2 * (sonar-> range + sonar_obst_depth);
        fov_pts.push_back(fov_us_l_depth);

        geometry_msgs::PointStamped fov_us_c_depth;
        fov_us_c_depth.header = sonar->header;
        fov_us_c_depth.point.x = sonar->range + sonar_obst_depth;
        fov_pts.push_back(fov_us_c_depth);

        geometry_msgs::PointStamped fov_us_r_depth;
        fov_us_r_depth.header = sonar->header;
        fov_us_r_depth.point.x = fov_us_l_depth.point.x;
        fov_us_r_depth.point.y = -fov_us_l_depth.point.y;
        fov_pts.push_back(fov_us_r_depth);
    }
}

/*
 * Helper to clearSonar() and markSonar()
 * Transform PointStamped's contained in fov_us_pts into the map_frame, adding
 * them to fov_odom_pts.
 *
 * Precondition: fov_us_pts should contain at least one item.
 *
 * Returns true if successful, false if a tf::TransformException was raised.
 */
bool transformPolyPts(
    const std::vector<geometry_msgs::PointStamped>& fov_us_pts,
    std::vector<geometry_msgs::PointStamped>& fov_odom_pts) {

    try {
        tf_l->waitForTransform(map_frame,
                               fov_us_pts.at(0).header.frame_id,
                               fov_us_pts.at(0).header.stamp,
                               ros::Duration(0.2));


        for (const auto& us_pt : fov_us_pts) {
            geometry_msgs::PointStamped odom_pt;
            tf_l->transformPoint(map_frame, us_pt, odom_pt);
            fov_odom_pts.push_back(odom_pt);
        }

    } catch (tf::TransformException &e) {
        ROS_ERROR("%s", e.what());
        return false;
    }

    return true;
}

/*
 * Helper to sonarHandler()
 * Clear the grid map area for a given sonar measurement.
 */
void clearSonar(const sensor_msgs::Range::ConstPtr& sonar) {
    std::vector<geometry_msgs::PointStamped> fov_us_pts;
    polygonFovPts(sonar, fov_us_pts);

    std::vector<geometry_msgs::PointStamped> fov_odom_pts;

    if (!transformPolyPts(fov_us_pts, fov_odom_pts)) {
        return;
    }

    grid_map::Polygon clear_poly;
    clear_poly.setFrameId(rover_map.getFrameId());
    clear_poly.addVertex(grid_map::Position(currentLocation.x,
                                            currentLocation.y));

    for (const auto& odom_pt : fov_odom_pts) {
        clear_poly.addVertex(grid_map::Position(odom_pt.point.x,
                                                odom_pt.point.y));
    }

    grid_map::Matrix& obstacle_layer = rover_map["obstacle"];

    for (grid_map::PolygonIterator iterator(rover_map, clear_poly);
         !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        double val = obstacle_layer(index(0), index(1));
        obstacle_layer(index(0), index(1)) = decreaseVal(val, 0.05);
    }
}

/*
 * Helper to sonarHandler()
 * Mark the grid map area for a given sonar measurement.
 */
void markSonar(const sensor_msgs::Range::ConstPtr& sonar) {
    std::vector<geometry_msgs::PointStamped> fov_us_pts;
    polygonFovPts(sonar, fov_us_pts, true);

    std::vector<geometry_msgs::PointStamped> fov_odom_pts;

    if (!transformPolyPts(fov_us_pts, fov_odom_pts)) {
        return;
    }

    grid_map::Polygon mark_poly;
    mark_poly.setFrameId(rover_map.getFrameId());

    for (const auto& odom_pt : fov_odom_pts) {
        mark_poly.addVertex(grid_map::Position(odom_pt.point.x,
                                               odom_pt.point.y));
    }

    grid_map::Matrix& obstacle_layer = rover_map["obstacle"];

    for (grid_map::PolygonIterator iterator(rover_map, mark_poly);
         !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);

        obstacle_layer(index(0), index(1)) = increaseVal(
            obstacle_layer(index(0), index(1)),
            sonar_base_mark_rate * (3.0 / sonar->range)
        );
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
void sonarHandler(
    const sensor_msgs::Range::ConstPtr& sonarLeft,
    const sensor_msgs::Range::ConstPtr& sonarCenter,
    const sensor_msgs::Range::ConstPtr& sonarRight) {

    if (!params_configured) {
        return;
    }

    static unsigned int prev_status = 0;
    unsigned int next_status = 0;

    // Minimum distance sonar center obstacles will be marked at. Anything
    // inside this distance should be a block in the claw.
    const double MIN_CENTER_DIST = 0.15;

    // Update the timestamp in the Obstacle map.
    rover_map.setTimestamp(ros::Time::now().toNSec());

    // Calculate the obstacle status.
    // Single sensor ranges below single sensor threshold can trigger an
    // obstacle message.
    if (sonarLeft->range < single_sensor_obst_dist) {
        next_status |= swarmie_msgs::Obstacle::SONAR_LEFT;
    }
    if (sonarCenter->range < single_sensor_obst_dist &&
        sonarCenter->range > 0.12) {
        next_status |= swarmie_msgs::Obstacle::SONAR_CENTER;
    }
    if (sonarRight->range < single_sensor_obst_dist) {
        next_status |= swarmie_msgs::Obstacle::SONAR_RIGHT;
    }

    // Two adjacent sensors both below double sensor threshold can also trigger
    // an obstacle message.
    if (sonarLeft->range < double_sensor_obst_dist &&
        sonarCenter->range < double_sensor_obst_dist) {
        next_status |= swarmie_msgs::Obstacle::SONAR_LEFT;
        next_status |= swarmie_msgs::Obstacle::SONAR_CENTER;
    }
    if (sonarRight->range < double_sensor_obst_dist &&
        sonarCenter->range < double_sensor_obst_dist) {
        next_status |= swarmie_msgs::Obstacle::SONAR_RIGHT;
        next_status |= swarmie_msgs::Obstacle::SONAR_CENTER;
    }

    if (sonarCenter->range < 0.12) {
        //block in front of center ultrasound.
        next_status |= swarmie_msgs::Obstacle::SONAR_BLOCK;
    }


    clearSonar(sonarLeft);

    // Only use if its range is far enough away to definitely not be a block
    // in the claw.
    if (sonarCenter->range > MIN_CENTER_DIST) {
        clearSonar(sonarCenter);
    }

    clearSonar(sonarRight);

    if (sonarLeft->range < sonar_view_range) {
        markSonar(sonarLeft);
    }

    if (sonarCenter->range > MIN_CENTER_DIST && sonarCenter->range < sonar_view_range) {
        markSonar(sonarCenter);
    }

    if (sonarRight->range < sonar_view_range) {
        markSonar(sonarRight);
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
void targetHandler(const apriltags2to1::AprilTagDetectionArray::ConstPtr& message) {
    if (!params_configured) {
        return;
    }

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
        grid_map::Position(
            currentLocation.x
                + CAMERA_NEAR_DIST * cos(currentLocation.theta + CAMERA_NEAR_ANGLE),
            currentLocation.y
                + CAMERA_NEAR_DIST * sin(currentLocation.theta + CAMERA_NEAR_ANGLE)
        )
    );

    // Far left corner
    view_poly.addVertex(
        grid_map::Position(
            currentLocation.x
                + CAMERA_FAR_DIST * cos(currentLocation.theta + CAMERA_FAR_ANGLE),
            currentLocation.y
                + CAMERA_FAR_DIST * sin(currentLocation.theta + CAMERA_FAR_ANGLE)
        )
    );

    // Far right corner
    view_poly.addVertex(
        grid_map::Position(
            currentLocation.x
                + CAMERA_FAR_DIST * cos(currentLocation.theta - CAMERA_FAR_ANGLE),
            currentLocation.y
                + CAMERA_FAR_DIST * sin(currentLocation.theta - CAMERA_FAR_ANGLE)
        )
    );

    // Near right corner
    view_poly.addVertex(
        grid_map::Position(
            currentLocation.x
                + CAMERA_NEAR_DIST * cos(currentLocation.theta - CAMERA_NEAR_ANGLE),
            currentLocation.y
                + CAMERA_NEAR_DIST * sin(currentLocation.theta - CAMERA_NEAR_ANGLE)
        )
    );

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
            tf_l->waitForTransform(
                map_frame,   // Target frame
                message->detections[0].pose.header.frame_id, // Source frame
                message->detections[0].pose.header.stamp,    // Time
                ros::Duration(0.2) // How long to wait for the tf.
            );

            for (int i=0; i<message->detections.size(); i++) {
                geometry_msgs::PoseStamped tagpose;
                tf_l->transformPose(map_frame,
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
        msg.mask = swarmie_msgs::Obstacle::TAG_TARGET | swarmie_msgs::Obstacle::TAG_HOME;
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
        tolerance = int(req.tolerance / rover_map.getResolution());
    }

    std::map<GridLocation, GridLocation> came_from;
    std::map<GridLocation, double> cost_so_far;

    bool success = a_star_search(rover_map, start, goal, tolerance,
                                 use_home_layer, came_from, cost_so_far);

    if (visualize_frontier) {
        rover_map.clear("frontier");
        grid_map::Index frontier_index;

        for (auto const &item : came_from) {
            frontier_index(0) = item.first.x;
            frontier_index(1) = item.first.y;
            rover_map.at("frontier", frontier_index) = 1.0;
        }
    }

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

/*
 * Subscriber to help with testing/debugging. Responds to RViz nav_goals
 * published with a mouse click and gets path from the rover's current location
 * to the goal location.
 */
void navGoalHandler(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    mapping::GetNavPlan::Request request;
    mapping::GetNavPlan::Response response;

    bool use_home_layer = false;
    ros::param::param<bool>("use_home_layer", use_home_layer, false);
    if (use_home_layer) {
        ROS_INFO("Using home layer in this path search.");
    }

    request.start.pose.position.x = currentLocation.x;
    request.start.pose.position.y = currentLocation.y;
    request.goal.pose.position.x = goal->pose.position.x;
    request.goal.pose.position.y = goal->pose.position.y;
    request.use_home_layer.data = use_home_layer;

    get_plan(request, response);
}

void crashHandler(int s) {
    int j, nptrs;
    void *buffer[1000];

    nptrs = backtrace(buffer, 1000);
    printf("backtrace() returned %d addresses\n", nptrs);
    backtrace_symbols_fd(buffer, nptrs, STDOUT_FILENO);
    exit(-s);
}

/*
 * Reconfigure obstacle mapping and path search parameters.
 */
void reconfigure(mapping::mappingConfig& cfg, uint32_t level) {
    sonar_fov = cfg.groups.map.sonar_fov;
    cos_fov_2 = cos(sonar_fov / 2.0);
    sin_fov_2 = sin(sonar_fov / 2.0);

    single_sensor_obst_dist = cfg.groups.map.single_sensor_obstacle_dist;
    double_sensor_obst_dist = cfg.groups.map.double_sensor_obstacle_dist;
    sonar_view_range = cfg.groups.map.sonar_view_range;
    sonar_obst_depth = cfg.groups.map.sonar_obstacle_depth;
    sonar_base_mark_rate = cfg.groups.map.sonar_base_mark_rate;
    sonar_base_clear_rate = cfg.groups.map.sonar_base_clear_rate;

    obstacle_threshold = cfg.groups.search.obstacle_threshold;
    inflation_pct = cfg.groups.search.inflation_pct;
    lethal_cost = cfg.groups.search.lethal_cost;
    neutral_cost = cfg.groups.search.neutral_cost;

    params_configured = true;
    ROS_INFO_THROTTLE(1, "Reconfigured mapping parameters.");
}

void initialconfig() {
    mapping::mappingConfig cfg;

    ros::param::get("~sonar_fov", cfg.sonar_fov);
    ros::param::get("~single_sensor_obstacle_dist", cfg.single_sensor_obstacle_dist);
    ros::param::get("~double_sensor_obstacle_dist", cfg.double_sensor_obstacle_dist);
    ros::param::get("~sonar_view_range", cfg.sonar_view_range);
    ros::param::get("~sonar_obstacle_depth", cfg.sonar_obstacle_depth);
    ros::param::get("~sonar_base_mark_rate", cfg.sonar_base_mark_rate);
    ros::param::get("~sonar_base_clear_rate", cfg.sonar_base_clear_rate);

    ros::param::get("~obstacle_threshold", cfg.obstacle_threshold);
    ros::param::get("~inflation_pct", cfg.inflation_pct);
    ros::param::get("~lethal_cost", cfg.lethal_cost);
    ros::param::get("~neutral_cost", cfg.neutral_cost);

    dynamic_reconfigure::Client<mapping::mappingConfig> client("mapping");

    if (client.setConfiguration(cfg)){
        ROS_INFO("Sent initial mapping config.");
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mapping");
    ros::NodeHandle mNH;

    signal(SIGSEGV, crashHandler);

    obstacle_status = swarmie_msgs::Obstacle::PATH_IS_CLEAR;

    // Parameters
    ros::param::param<std::string>("odom_frame", map_frame, "odom");

    double map_size;
    double map_resolution;
    ros::param::param<double>("~map_size", map_size, 25.0);
    ros::param::param<double>("~map_resolution", map_resolution, 0.5);
    ros::param::param<bool>("~visualize_frontier", visualize_frontier, false);

    // Setup dynamic reconfigure server, which also automatically reads any
    // initial parameters put on the parameter server at startup.
    dynamic_reconfigure::Server<mapping::mappingConfig> config_server;
    dynamic_reconfigure::Server<mapping::mappingConfig>::CallbackType f;
    f = boost::bind(&reconfigure, _1, _2);
    config_server.setCallback(f);

    boost::thread t(initialconfig);

    // Transform Listener
    //
    // C++ Tutorial Here:
    // http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
    tf_l = new tf::TransformListener(ros::Duration(10));

    // Subscribers
    //
    // C++ Tutorial Here:
    // http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
    //
    ros::Subscriber odomSubscriber = mNH.subscribe("odom/filtered", 10, odometryHandler);
    ros::Subscriber targetSubscriber = mNH.subscribe("targets", 10, targetHandler);
    ros::Subscriber goalSubscriber = mNH.subscribe("goal", 10, navGoalHandler);

    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, "sonarLeft", 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, "sonarCenter", 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, "sonarRight", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    // Publishers

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
    rover_map = grid_map::GridMap({"obstacle", "target", "home", "frontier"});
    rover_map.setFrameId(map_frame);

    if (visualize_frontier) {
        ROS_INFO("Visualizing A* search frontier.");
    }

    ROS_INFO("Initializing %f m x %f m map with resolution %f m per cell",
             map_size,
             map_size,
             map_resolution);

    rover_map.setGeometry(grid_map::Length(map_size, map_size), map_resolution);

    ros::spin();
    return 0;
}
