#include <deque>
#include "rover_msgs/Waypoint.lcm"
#include "string"

namespace WaypointTypes{
    const std::string SEARCH;
    const std::string DESTINATION;
    const std::string GATE_TRAVERSAL;
}


// TEST COMMENT
class Course {
    public:

    Course();

    void pushFront(Waypoint waypoint);

    void pushBack(Waypoint waypoint);

    void popFront();

    void popBack();

    void clearAllOfType(std::string waypointType);

    private:

    std::deque<Waypoint> waypoints;

};