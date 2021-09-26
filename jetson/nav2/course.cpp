#include "course.hpp"

const std::string WaypointTypes::SEARCH = "search";
const std::string WaypointTypes::GATE_TRAVERSAL = "gateTraversal";
const std::string WaypointTypes::DESTINATION = "destination";


void Course::pushFront(Waypoint waypoint){
    this->waypoints.push_front(waypoint);
}

void Course::pushBack(Waypoint waypoint){
    this->waypoints.push_back(waypoint);
}

void Course::popFront(){
    this->waypoints.pop_front();
}

void Course::popBack(){
    this->waypoints.pop_back();
}

void Course::clearAllOfType(std::string waypointType){
    // delete waypoints of type waypointType starting from end
    for (auto it = this->waypoints.end(); it != this->waypoints.begin(); --it){
        if (it->type == waypointType){
            this->waypoints.erase(it);
        }
    }
}

Waypoint& Course::peekTop(){
    return this->waypoints[0];
}