#include <deque>
#include <utility>
#include "waypoint.hpp"
#include "string"

// TEST COMMENT
class Course {
    public:

    Course();

    void pushFront(Waypoint waypoint);

    void pushBack(Waypoint waypoint);

    void popFront();

    void popBack();

    void clearAllOfType(WaypointType waypointType);

    Waypoint& peekTop();

    private:

    void calculateEnds();

    std::deque<std::pair<int, Waypoint>> &getDeque(WaypointType);

    std::deque<std::pair<int, Waypoint>> &getFrontDeque();

    std::deque<std::pair<int, Waypoint>> &getLastDeque();

    int first;

    int last;

    std::deque<std::pair<int, Waypoint>> waypointsSearch;

    std::deque<std::pair<int, Waypoint>> waypointsGateTraversal;

    std::deque<std::pair<int, Waypoint>> waypointsDestination;

    std::deque<std::pair<int, Waypoint>> waypointsTargetApproach;
};