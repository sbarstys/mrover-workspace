#include <string>
#include "rover_msgs/Odometry.lcm"


enum class WaypointType
{
    // Base States
    Search = 0,
    GateTraversal = 1,
    Destination = 2,
    TargetApproach = 3,
};

class Waypoint {
    public:
    
        Waypoint(bool search_in, bool gate_in, float gate_width_in, int id_in, Odometry odom_in, std::string type_in);

        bool search;

        bool gate;

        float gate_width;

        int id;

        Odometry odom;

        WaypointType type;

    private:

};