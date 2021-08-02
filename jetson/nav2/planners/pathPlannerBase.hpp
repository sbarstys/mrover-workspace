#include "path.hpp"
#include "rover_msgs/Waypoint.lcm"

class PathPlannerBase{

    public:
    virtual Path& generatePath(Waypoint& target) = 0;

    protected:
    Path m_path;

};