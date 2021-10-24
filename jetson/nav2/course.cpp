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
//Determine the values of the first and last ID of the deque
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
    : first(0), last(-1), lastTypeUsed(WaypointType.Search) {}

//TO DO Evaluate if last waypoint type is needed (lastTypeUsed, line 79 and others)

//Return the deque whose waypoint corresponds to the first element in the original deque
deque<pair<int, Waypoint>> &Course::getFrontDeque() {
    // Check if the queue we need was used recently
    if (getDeque(lastTypeUsed).front().first == first) return getDeque(lastTypeUsed);

    // We know the first index holds the first item, and it is at the front of some deque
    // Therefore, we can just look for which deque starts with this 'first' number
    if (waypointsSearch.front().first == first) return waypointsSearch;
    else if (waypointsGateTraversal.front().first == first) return waypointsGateTraversal;
    else if (waypointsDestination.front().first == first) return waypointsDestination;
    else if (waypointsTargetApproach.front().first == first) return waypointsTargetApproach;
    else cout << "No Waypoint deques are at the true front";
    return waypointsTargetApproach;
}

//Return the deque whose waypoint corresponds to the last element in the original deque
deque<pair<int, Waypoint>> &Course::getLastDeque() {
    // Check if the queue we need was used recently
    if (getDeque(lastTypeUsed).back().first == last) return getDeque(lastTypeUsed);

    // We know the last index holds the last item, and it is at the back of some deque
    // Therefore, we can just look for which deque ends with this 'last' number
    if (waypointsSearch.back().first == last) return waypointsSearch;
    else if (waypointsGateTraversal.back().first == last) return waypointsGateTraversal;
    else if (waypointsDestination.back().first == last) return waypointsDestination;
    else if (waypointsTargetApproach.back().first == last) return waypointsTargetApproach;
    else cout << "No Waypoint deques are at the true back";
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

    // Update lastTypedUsed so this WaypointType is more recent
    lastTypeUsed = waypoint.type;
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
    
    // Update lastTypedUsed so this WaypointType is more recent
    lastTypeUsed = waypoint.type;
}

// Remove the first waypoint
void Course::popFront(){
    deque<pair<int, Waypoint>> waypoints = getFrontDeque();
    //Increment first so that it now points to this new waypoint
    first++;
    // Update lastTypedUsed so this WaypointType is more recent
    lastTypeUsed = waypoints.front().first.type;
    //Actually remove it from the deque
    waypoints.pop_front();
}

//Remove the last waypoint
void Course::popBack(){
    deque<pair<int, Waypoint>> waypoints = getLastDeque();
    //Decrease last, so that it now points to this new waypoint
    last--;
    // Update lastTypedUsed so this WaypointType is more recent
    lastTypeUsed = waypoints.back().first.type;
    //Actually remove it from the deque
    waypoints.pop_back();
}

// clear the deque for this type
void Course::clearAllOfType(WaypointType waypointType){
    deque<pair<int, Waypoint>> waypoints = getDeque(waypointType);
    waypoints.clear();
    calculateEnds();
}

//Returns the first item's waypoint.
Waypoint& Course::front(){
    return getFrontDeque().front().second;
}

//Returns the last item's waypoint.
Waypoint& Course::back(){ 
    return getFrontDeque().back().second;
}