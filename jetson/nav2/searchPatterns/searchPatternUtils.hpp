#ifndef SEARCHPATTERNUTILS
#define SEARCHPATTERNUTILS
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/Waypoint.hpp"
#include "rover.hpp"
#include "utilities.hpp"
void insertWaypointsIntoCourse(Waypoint p1, Waypoint p2, double visionDistance);

#endif