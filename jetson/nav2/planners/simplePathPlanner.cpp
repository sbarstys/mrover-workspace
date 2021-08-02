#include "simplePathPlanner.hpp"

// Generate basic path by adding one waypoint, and returning the path
Path& SimplePathPlanner::generatePath(Waypoint& target){
    Odometry endpoint;
    endpoint = target.odom;
    m_path.clear();
    m_path.addPoint(endpoint);
    return m_path;
}