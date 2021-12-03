#include "pathFollowerBase.hpp"
#include "path.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/Waypoint.hpp"
#include "math.h"
#include "rover.hpp"
#include "utilities.hpp"


class SimplePathFollower : public PathFollowerBase{
    public:
        SimplePathFollower(lcm::LCM& lcmObject);

    private:
    void followPath(Path& path);
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
        DriveState executeDrive(std::vector<Odometry>& path);
        DriveState executeTurn(std::vector<Odometry>& path);
        DriveState executeTurnAroundObs(std::vector<Odometry>&  path);
        DriveState executeDriveAroundObs(std::vector<Odometry>& path);


        void updateObstacleAngle( double bearing );
        void updateObstacleDistance( double distance );
        void updateObstacleElements( double bearing, double distance );
        bool isWaypointReachable( double distance );
        Odometry createAvoidancePoint( Rover* rover, const double distance );
};



