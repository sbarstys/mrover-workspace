#include "waypoint.hpp"
#include "rover_msgs/Odometry.lcm"

using namespace std;

WaypointType parseType(string type) {
    if (type == "search") return WaypointType::Search;
    if (type == "gateTraversal") return WaypointType::GateTraversal;
    if (type == "destination") return WaypointType::Destination;
    if (type == "targetApproach") return WaypointType::TargetApproach;
}

Waypoint::Waypoint(bool search_in, bool gate_in, float gate_width_in, int id_in, 
    Odometry odom_in, string type_in)
    : search(search_in), gate(gate_in), gate_width(gate_width_in), id(id_in), odom(odom_in), type(parseType(type_in))
    { }

