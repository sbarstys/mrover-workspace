#include <deque>
#include <utility>
#include "course.hpp"
#include "waypoint.hpp"

using namespace std;

// const std::string WaypointTypes::SEARCH = "search";
// const std::string WaypointTypes::GATE_TRAVERSAL = "gateTraversal";
// const std::string WaypointTypes::DESTINATION = "destination";

//Returns the deque corresponding to the passed Waypoint Type
deque<pair<int, Waypoint>> &Course::getDeque(WaypointType type) {
    switch(type) {
        case WaypointType::Search:
            return waypointsSearch;
        case WaypointType::GateTraversal:
            return waypointsGateTraversal;
        case WaypointType::Destination:
            return waypointsDestination;
        case WaypointType::TargetApproach:
            return waypointsTargetApproach;
    }
}

void Course::calculateEnds() {
    // We are finding the min and max, so set it to the 
    // worst possible cases so the first element will override this
    // (this is a common practice when finding a maximum/minimum)
    int firstIndex = INT_MAX;
    int lastIndex = INT_MIN;

    // To avoid rewriting code, put each deque we are looking through in a list
    deque<pair<int, Waypoint>> *loop[4] = {
        &waypointsSearch,
        &waypointsGateTraversal,
        &waypointsDestination,
        &waypointsTargetApproach,
    };

    // Loop through each deque to find the first and last items
    for(int i = 0; i < 4; i++) {
        deque<pair<int, Waypoint>> waypointDeque = *(loop[i]);
        // Make sure this deque actually has something
        if (!waypointDeque.empty()) {
            // Get the index of the front (for first)
            int frontIndex = waypointDeque.front().first;
            // Get the index of the back (for last)
            int backIndex = waypointDeque.back().first;

            // Check if either of these is the new first or last
            if (frontIndex < firstIndex) {
                firstIndex = frontIndex;
            }
            if (backIndex > lastIndex) {
                lastIndex = lastIndex;
            }
        }
    }
    first = firstIndex;
    last = lastIndex;
    
    // If the deque is empty, then firstIndex and lastIndex would have never
    // been updated. In this case, set first to 0 and last to -1
    if (firstIndex == INT_MAX) {
        first = 0;
        last = -1;
    }
}

//Initializes first to 0 and last to -1 (so adding one element fixes the indices)
// The deques do not have to be initialized
Course::Course() 
    : first(0), last(-1) {}

deque<pair<int, Waypoint>> &Course::getFrontDeque() {
    // We know the first index holds the first item, and it is at the front of some deque
    // Therefore, we can just look for which deque starts with this 'first' number
    if (waypointsSearch.front().first == first) return waypointsSearch;
    if (waypointsGateTraversal.front().first == first) return waypointsGateTraversal;
    if (waypointsDestination.front().first == first) return waypointsDestination;
    if (waypointsTargetApproach.front().first == first) return waypointsTargetApproach;
    return waypointsTargetApproach;
}

deque<pair<int, Waypoint>> &Course::getLastDeque() {
    // We know the last index holds the last item, and it is at the back of some deque
    // Therefore, we can just look for which deque ends with this 'last' number
    if (waypointsSearch.back().first == last) return waypointsSearch;
    if (waypointsGateTraversal.back().first == last) return waypointsGateTraversal;
    if (waypointsDestination.back().first == last) return waypointsDestination;
    if (waypointsTargetApproach.back().first == UINT_LEAST8_MAX) return waypointsTargetApproach;
    return waypointsTargetApproach;
}


// Add an item to the front
void Course::pushFront(Waypoint waypoint){
    // Get the deque this waypoint belongs to
    deque<pair<int, Waypoint>> waypoints = getDeque(waypoint.type);
    // Decrease first, so that it now points to this new waypoint
    first--;
    // Create the pair that holds this waypoint. It's index is first (since this is pushFront)
    pair<int, Waypoint> pair (first, waypoint);
    // Actually add it to the deque
    waypoints.push_front(pair);
}

//Add an item to the back 
void Course::pushBack(Waypoint waypoint){
    //Get the deque this waypoint belongs to
    deque<pair<int, Waypoint>> waypoints = getDeque(waypoint.type);
    //Increase last, so that it now points to this new waypoint
    last++;
    // Create the pair that holds this waypoint.  It's index is last (since this is pushBack)
    pair<int, Waypoint> pair (last, waypoint);
    //Actually add it to the deque.
    waypoints.push_back(pair);
}

// Remove the first waypoint
void Course::popFront(){
    deque<pair<int, Waypoint>> waypoints = getFrontDeque();
    //Increment first so that it now points to this new waypoint
    first++;
    //Actually remove it from the deque
    waypoints.pop_front();
}

//Remove the last waypoint
void Course::popBack(){
    deque<pair<int, Waypoint>> waypoints = getLastDeque();
    //Decrease last, so that it now points to this new waypoint
    last--;
    //Actually remove it from the deque
    waypoints.pop_back();
}

void Course::clearAllOfType(WaypointType waypointType){
    // clear the deque for this type
    deque<pair<int, Waypoint>> waypoints = getDeque(waypointType);
    waypoints.clear();
    calculateEnds();
}

Waypoint& Course::peekTop(){
    //Returns the first item's waypoint.
    return getFrontDeque().front().second;
}