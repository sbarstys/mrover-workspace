#include "pathFollowerBase.hpp"
#include "path.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/Waypoint.hpp"
#include "math.h"
#include "rover.hpp"
#include "utilities.hpp"

class SimplePathFollower : public PathFollowerBase{
    public:
        void followPath(Path& path);
        SimplePathFollower(lcm::LCM& lcmObject);

    private:
        enum class DriveStatus {
            //Temporary drive statuses
            OnCourse = 1,
            OffCourse = 2, 
        }

        enum class DriveState {
            //Done State
            Done = 0,
            
            //Drive States
            Drive = 10,

            //Obstacle Avoidance States
            TurnAroundObs = 20,
            DriveAroundObs = 21,

            //Turn State
            Turn = 30,
        }
        Odometry mObstacleAvoidancePoint;
        // Initial angle to go around obstacle upon detection.
        double mOriginalObstacleAngle;
        // Initial angle to go around obstacle upon detection.
        double mOriginalObstacleDistance;
        // Current state for the DriveState pseudo state machine
        int current_state;
        DriveState executeDrive();
        DriveState executeTurn();
        DriveState executeTurnAroundObs();
        DriveState executeDriveAroundObs();
        DriveState executeTurn();

        void updateObstacleAngle( double bearing );
        void updateObstacleDistance( double distance );
        void updateObstacleElements( double bearing, double distance );
        bool isWaypointReachable( double distance );
        Odometry createAvoidancePoint( Rover* rover, const double distance );
};


