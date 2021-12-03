#ifndef PATH_FOLLOWER_BASE
#define PATH_FOLLOWER_BASE

#include "rover_msgs/Waypoint.hpp"
#include "rover_msgs/Odometry.hpp"
#include <vector>
#include "path.hpp"
#include "utilities.hpp"

class PathFollowerBase{
    public:
    
    virtual void followPath(Path& path) = 0;

    virtual bool isArrived(Waypoint waypoint) = 0;

    virtual ~PathFollowerBase() = default;

    protected:

};

#endif