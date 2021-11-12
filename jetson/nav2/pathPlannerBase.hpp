#ifndef PATH_PLANNER_BASE
#define PATH_PLANNER_BASE

#include "rover_msgs/Waypoint.hpp"
#include "rover_msgs/Odometry.hpp"
#include <vector>
#include "utilities.hpp"

class PathPlannerBase{

    public:
    virtual std::vector<Odometry>& generatePath(Waypoint& target) = 0;

    virtual ~PathPlannerBase() = default;

    protected:
    std::vector<Odometry> m_path;

};

#endif