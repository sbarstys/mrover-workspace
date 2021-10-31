#ifndef SIMPLE_PATH_PLANNER_HPP
#define SIMPLE_PATH_PLANNER_HPP

#include "pathPlannerBase.hpp";
#include "path.hpp";
#include "rover_msgs/Odometry.hpp"

class SimplePathPlanner : public PathPlannerBase{
    public:
    Path& generatePath(Waypoint& target);
};

#endif