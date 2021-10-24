#include <deque>
#include <utility>
#include "waypoint.hpp"
#include "string"

// TEST COMMENT
class Course {
    public:

    Course();
    // Add an item to the front
    void pushFront(Waypoint waypoint);

    //Add an item to the back 
    void pushBack(Waypoint waypoint);

    // Remove the first waypoint
    void popFront();

    //Remove the last waypoint
    void popBack();

    // clear the deque for this type
    void clearAllOfType(WaypointType waypointType);

    //Returns the first item's waypoint.
    Waypoint& front();

    //Returns the last item's waypoint.
    Waypoint& back();

    private:
    //Determine the values of the first and last ID of the deque
    void calculateEnds();

    //Returns the deque corresponding to the passed Waypoint Type
    std::deque<std::pair<int, Waypoint>> &getDeque(WaypointType);

    //Return the deque whose waypoint corresponds to the first element in the original deque
    std::deque<std::pair<int, Waypoint>> &getFrontDeque();

    //Return the deque whose waypoint corresponds to the last element in the original deque
    std::deque<std::pair<int, Waypoint>> &getLastDeque();

    //An ID of the first element in the deque
    int first;

    //An ID of the last element in the deque
    int last;

    //A deque containing a pair object that holds the ID of the Search waypoint in the original deque
    std::deque<std::pair<int, Waypoint>> waypointsSearch;

    //A deque containing a pair object that holds the ID of the GateTraversal waypoint in the original deque
    std::deque<std::pair<int, Waypoint>> waypointsGateTraversal;

    //A deque containing a pair object that holds the ID of the Destination waypoint in the original deque
    std::deque<std::pair<int, Waypoint>> waypointsDestination;

    //A deque containing a pair object that holds the ID of the Target waypoint in the original deque
    std::deque<std::pair<int, Waypoint>> waypointsTargetApproach;

    //A waypoint type variable storing the last WaypointType used
    WaypointType lastTypeUsed;
};