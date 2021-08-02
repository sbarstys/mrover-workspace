#include "rover_msgs/Waypoint.lcm"
#include "path.hpp"

class PathFollowerBase{
    public:
    
    virtual void followPath(Path& path) = 0;

    bool isArrived(Waypoint& waypoint);

    protected:

};