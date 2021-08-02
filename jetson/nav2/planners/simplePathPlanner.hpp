#include "pathPlannerBase.hpp";
#include "path.hpp";
#include "rover_msgs/Odometry.lcm"


class SimplePathPlanner : PathPlannerBase{
    public:
    Path& generatePath(Waypoint& target);
};