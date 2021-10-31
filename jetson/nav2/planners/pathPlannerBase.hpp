#ifndef PATH_PLANNER_BASE
#define PATH_PLANNER_BASE

#include "path.hpp"
#include "rover_msgs/Waypoint.hpp"

class PathPlannerBase{

    public:
    virtual Path& generatePath(Waypoint& target) = 0;

    virtual ~PathPlannerBase() = default;

    protected:
    Path m_path;

};

#endif