#include "pathFollowerBase.hpp"
#include "path.hpp"
#include "rover_msgs/Odometry.lcm"
#include "rover_msgs/Waypoint.lcm"
#include "math.h"

class SimplePathFollower : PathFollowerBase{
    void followPath(Path& path);
};