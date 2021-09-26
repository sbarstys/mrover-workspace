#include <deque>
#include "rover_msgs/Waypoint.lcm"
#include "string"

namespace WaypointTypes{
    const std::string SEARCH;
    const std::string DESTINATION;
    const std::string GATE_TRAVERSAL;
}



class Course {
    public:

    Course();

    void pushFront(Waypoint waypoint);

    void pushBack(Waypoint waypoint);

    void popFront();

    void popBack();

    void clearAllOfType(std::string waypointType);

    Waypoint& peekTop();

    private:

    std::deque<Waypoint> waypoints;

};