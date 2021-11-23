#include "simplePathPlanner.hpp"
#include <vector>

// Generate basic path by adding one waypoint, and returning the path
std::vector<Odometry>& SimplePathPlanner::generatePath(Waypoint& target){
    Odometry endpoint;
    endpoint = target.odom;
    m_path.clear();
    m_path.push_back(endpoint);
    return m_path;
}