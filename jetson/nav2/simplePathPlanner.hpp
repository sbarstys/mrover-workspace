#ifndef SIMPLE_PATH_PLANNER_HPP
#define SIMPLE_PATH_PLANNER_HPP

#include "pathPlannerBase.hpp"
#include "rover_msgs/Odometry.hpp"
#include <vector>

class SimplePathPlanner : public PathPlannerBase{
    public:
    std::vector<Odometry>& generatePath(Waypoint& target);
};

#endif