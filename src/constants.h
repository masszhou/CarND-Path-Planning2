//
// Created by zzhou on 16.01.18.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

// given map information
#define MAP_TOTAL_WAYPOINT 181
#define MAP_S_LENGTH 6945.554       // track length = 6945.544 meters
#define MAP_LANE_WIDTH 4            // meters
#define LANES_NUMBER                // how many lanes
#define SENSOR_RANGE 200 //meter


#define SIM_DT 0.02  // [s]
#define MAX_SPEED 22 // [m/s]
#define MAX_JERK 10 // [m/s/s/s], due to project specification
#define MAX_ACCEL 10 // [m/s]

// safety
#define CRUISE_FOLLOW_DIST 35 //m
#define TOO_CLOSE_WARNING_DIST 25 //m
#define FORWARD_COLLISION_WARNING_DIST 15 // m

#define TRAJECTORY_SIZE 50 // 50 points

#endif //PATH_PLANNING_CONSTANTS_H
