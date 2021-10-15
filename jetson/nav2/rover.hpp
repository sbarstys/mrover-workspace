#ifndef ROVER_HPP
#define ROVER_HPP

#include <lcm/lcm-cpp.hpp>
#include <queue>

#include "rover_msgs/AutonState.hpp"
#include "rover_msgs/Bearing.hpp"
#include "rover_msgs/Course.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/RepeaterDrop.hpp"
#include "rover_msgs/RadioSignalStrength.hpp"
#include "rover_msgs/TargetList.hpp"
#include "waypoint.hpp"
#include "course.hpp"
#include "rapidjson/document.h"
#include "pid.hpp"

using namespace rover_msgs;
using namespace std;


// This class creates a Rover object which can perform operations that
// the real rover can perform.
class Rover
{
public:
    // This class holds all the status informatin of the rover.
    class RoverStatus
    {
    public:
        struct PostLocation{
            Odometry location;
            int32_t id;
        };
        
        RoverStatus();

        RoverStatus(
            Bearing bearingIn,
            Course courseIn,
            Odometry odometryIn,
            Target targetIn,
            Target target2In,
            RadioSignalStrength signalIn
            );

        Destinations& destinations();

        Odometry& odometry();

        Target& target();

        Target& target2();

        bool firstGatePostFound();

        RadioSignalStrength& radio();

        RoverStatus& operator=( RoverStatus& newRoverStatus );

        Course& course();

        PostLocation post1();

        PostLocation post2();

    private:
        // The rover's overall course.
        Destinations mDestinations;

        // The rover's current odometry information.
        Odometry mOdometry;

        // The rover's current target information from computer
        // vision.
        Target mTarget1;

        Target mTarget2;

        // the rover's current signal strength to the base station
        RadioSignalStrength mSignal;

        Course mCourse;

        // first gate post found
        bool mFirstGatePostFound = false;

        //post locations
        PostLocation post1;
        PostLocation post2;

    };

    Rover( const rapidjson::Document& config, lcm::LCM& lcm_in );

    DriveStatus drive( const Odometry& destination );

    DriveStatus drive( const double distance, const double bearing, const bool target = false );

    void drive(const int direction, const double bearing);

    bool turn( Odometry& destination );

    bool turn( double bearing );

    void stop();

    bool updateRover( RoverStatus newRoverStatus );

    RoverStatus& roverStatus();

    PidLoop& distancePid();

    PidLoop& bearingPid();

    const double longMeterInMinutes() const;

    void updateRepeater( RadioSignalStrength& signal);

    bool isTimeToDropRepeater();

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    void publishJoystick( const double forwardBack, const double leftRight, const bool kill );

    bool isEqual( const Odometry& odometry1, const Odometry& odometry2 ) const;

    bool isEqual( const Target& target1, const Target& target2 ) const;

    bool isTurningAroundObstacle( const NavState currentState ) const;

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // The rover's current status.
    RoverStatus mRoverStatus;

    // A reference to the configuration file.
    const rapidjson::Document& mRoverConfig;

    // A reference to the lcm object that will be used for
    // communicating with the actual rover and the base station.
    lcm::LCM& mLcmObject;

    // The pid loop for driving.
    PidLoop mDistancePid;

    // The pid loop for turning.
    PidLoop mBearingPid;

    // If it is time to drop a radio repeater
    bool mTimeToDropRepeater;

    // The conversion factor from arcminutes to meters. This is based
    // on the rover's current latitude.
    double mLongMeterInMinutes;
};

Rover* gRover;

#endif // ROVER_HPP
