#include "pathFollowerBase.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/Waypoint.hpp"
#include "math.h"
#include <vector>

class SimplePathFollower : PathFollowerBase{
    void followPath(vector<Odometry>& path);
};