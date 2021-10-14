#include "stateMachine.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <map>

#include "rover_msgs/NavStatus.hpp"
#include "utilities.hpp"
#include "search/spiralOutSearch.hpp"
#include "search/spiralInSearch.hpp"
#include "search/lawnMowerSearch.hpp"
#include "obstacle_avoidance/simpleAvoidance.hpp"
#include "gate_search/diamondGateSearch.hpp"

// Constructs a StateMachine object with the input lcm object.
// Reads the configuartion file and constructs a Rover objet with this
// and the lcmObject. Sets mStateChanged to true so that on the first
// iteration of run the rover is updated.
StateMachine::StateMachine( lcm::LCM& lcmObject )
    : mRover( nullptr )
    , mLcmObject( lcmObject )
    , mTotalWaypoints( 0 )
    , mCompletedWaypoints( 0 )
    , mRepeaterDropComplete ( false )
    , mStateChanged( true )
{
    //Object to handle reading the configuration file
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_nav/config.json";
    configFile.open( configPath );
    string config = "";
    string setting;
    //Reads the configuration file
    while( configFile >> setting )
    {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse( config.c_str() );
    //Creates the mRover object with its attributes being the lcm object and data from the config file 
    mRover = new Rover( mRoverConfig, lcmObject );
    //Below are separe state machines that exist within the main statemachine to perform specific functions
    //Creates these separate state machines
    mSearchStateMachine = SearchFactory( this, SearchType::SPIRALOUT, mRover, mRoverConfig );
    mGateStateMachine = GateFactory( this, mRover, mRoverConfig );
    mObstacleAvoidanceStateMachine = ObstacleAvoiderFactory( this, ObstacleAvoidanceAlgorithm::SimpleAvoidance, mRover, mRoverConfig );
} // StateMachine()

// Destructs the StateMachine object. Deallocates memory for the Rover
// object.
StateMachine::~StateMachine( )
{
    delete mRover;
}

//Sets the search type the rover will be using
void StateMachine::setSearcher( SearchType type, Rover* rover, const rapidjson::Document& roverConfig )
{
    assert( mSearchStateMachine );
    delete mSearchStateMachine;
    mSearchStateMachine = SearchFactory( this, type, rover, roverConfig ); //Search object is set equal to the statemachien object
}

//Updates the completed waypoint variable
void StateMachine::updateCompletedPoints( )
{
    mCompletedWaypoints += 1;
    return;
}

void StateMachine::updateRepeaterComplete( )
{
    mRepeaterDropComplete = true;
    return;
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleAngle( double bearing )
{
    mObstacleAvoidanceStateMachine->updateObstacleAngle( bearing );
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleDistance( double distance )
{
    mObstacleAvoidanceStateMachine->updateObstacleDistance( distance );
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleElements( double bearing, double distance )
{
    updateObstacleAngle( bearing );
    updateObstacleDistance( distance );
}

// Runs the state machine through one iteration. The state machine will
// run if the state has changed or if the rover's status has changed.
// Will call the corresponding function based on the current state.
void StateMachine::run()
{
    publishNavState(); //Publishes the current nav state to the lcm channel to communicate with rest of rover
    if( isRoverReady() ) // If the rover is ready to run
    {
        mStateChanged = false;
        NavState nextState = NavState::Unknown;

        //If block handles the rover if it is in the off state from the beginning
        if( !mRover->roverStatus().autonState().is_auton ) //If the rover is not on
        {
            nextState = NavState::Off;
            mRover->roverStatus().currentState() = executeOff(); // turn off immediately.  Current state is the off state
            clear( mRover->roverStatus().path() ); //Clears the queue that stores the rover's path
            //If the current state is not off (on)
            if( nextState != mRover->roverStatus().currentState() )
            {
                //Set the current state to the off state
                mRover->roverStatus().currentState() = nextState;
                mStateChanged = true;
            }
            return;
        }
        //Evaluate what the next state and actions are
        switch( mRover->roverStatus().currentState() ) //Evaluates the current state of the rover
        //Current state is evaluated based on the data held by the rover status object 
        {
            case NavState::Off: //Is the rover currently in the off state?
            {
                //The next state is the state that the executeOff function returns (The off navestate)
                nextState = executeOff(); //Function returns the corresponding/correct state as an enum
                break;
            }

            case NavState::Done: //Is the rover currently in the done state?
            {
                nextState = executeDone(); //The Next state will be the one returned by the function. i.e. the Done state
                break;
            }
            //The following blocks of cases execute functions whose logic will make the next state something new
            case NavState::RadioRepeaterTurn: //Is the rover currently in the turn state and radiorepeaterturnstate?
            case NavState::Turn:
            {
                nextState = executeTurn(); //The next state is the nav state (returned from function)
                break;
            }

            case NavState::RadioRepeaterDrive: //Is the rover currently in the RadioRepeaterDrive and Drive state?
            case NavState::Drive:
            {
                nextState = executeDrive(); //The next state is the state returned from this function 
                break;
            }

            case NavState::RepeaterDropWait: //Is the rover in the RepeaterDropWaitState?
            {
                nextState = executeRepeaterDropWait();
                break;
            }

            //If your Nav State is any of these run the mSearchStateMachine.
            case NavState::SearchFaceNorth: 
            case NavState::SearchSpin:
            case NavState::SearchSpinWait:
            case NavState::SearchTurn:
            case NavState::SearchDrive:
            case NavState::TurnToTarget:
            case NavState::TurnedToTargetWait:
            case NavState::DriveToTarget:
            {
                nextState = mSearchStateMachine->run(); //This is not run() in stateMachine.cpp
                break;
            }

            //If your Nav State is in one of these states run the mObstacleAvoidanceStateMachine
            case NavState::TurnAroundObs: 
            case NavState::SearchTurnAroundObs:
            case NavState::DriveAroundObs:
            case NavState::SearchDriveAroundObs:
            {
                nextState = mObstacleAvoidanceStateMachine->run();
                break;
            }
            //If you are in the Nav State to change the search algorithm do this to change the search algorithm.
            case NavState::ChangeSearchAlg:
            {
                static int searchFails = 0;
                static double visionDistance = mRoverConfig[ "computerVision" ][ "visionDistance" ].GetDouble();

                //determine what search method should be chosen next
                switch( mRoverConfig[ "search" ][ "order" ][ searchFails % mRoverConfig[ "search" ][ "numSearches" ].GetInt() ].GetInt() )
                {
                    case 0:
                    {
                        //This is a type of search that gets set as the search method
                        setSearcher(SearchType::SPIRALOUT, mRover, mRoverConfig);
                        break;
                    }
                    case 1:
                    {
                        //This is a type of search that gets set as the search method
                        setSearcher(SearchType::LAWNMOWER, mRover, mRoverConfig);
                        break;
                    }
                    case 2:
                    {
                        //This is a type of search that gets set as the search method
                        setSearcher(SearchType::SPIRALIN, mRover, mRoverConfig);
                        break;
                    }
                    default:
                    {
                        //This is a type of search that gets set as the search method
                        //SpiralOut is the default search method
                        setSearcher(SearchType::SPIRALOUT, mRover, mRoverConfig);
                        break;
                    }
                }
                mSearchStateMachine->initializeSearch( mRover, mRoverConfig, visionDistance );
                if( searchFails % 2 == 1 && visionDistance > 0.5 )
                {
                    visionDistance *= 0.5;
                }
                searchFails += 1;
                nextState = NavState::SearchTurn;
                break;
            }
            //If your NavState is one of these movements then run the mGateStateMachine
            //This search deals with driving through the gate?
            case NavState::GateSpin:
            case NavState::GateSpinWait:
            case NavState::GateTurn:
            case NavState::GateDrive:
            case NavState::GateTurnToCentPoint:
            case NavState::GateDriveToCentPoint:
            case NavState::GateFace:
            case NavState::GateShimmy:
            case NavState::GateDriveThrough:
            {
                nextState = mGateStateMachine->run(); //mGateStateMachine calls its run function
                break;
            }
            //If the state is unknown terminate
            case NavState::Unknown:
            {
                cerr << "Entered unknown state.\n";
                exit(1);
            }
        } // switch

        //If there. If at the next state?
        if( nextState != mRover->roverStatus().currentState() )
        {
            mStateChanged = true;
            mRover->roverStatus().currentState() = nextState;
            mRover->distancePid().reset();
            mRover->bearingPid().reset();
        }
        cerr << flush;
    } // if
} // run()

// Updates the auton state (on/off) of the rover's status.
void StateMachine::updateRoverStatus( AutonState autonState )
{
    mNewRoverStatus.autonState() = autonState;
} // updateRoverStatus( AutonState )

// Updates the course of the rover's status if it has changed.
void StateMachine::updateRoverStatus( Course course )
{
    if( mNewRoverStatus.course().hash != course.hash )
    {
        mNewRoverStatus.course() = course;
    }
} // updateRoverStatus( Course )

// Updates the obstacle information of the rover's status.
void StateMachine::updateRoverStatus( Obstacle obstacle )
{
    mNewRoverStatus.obstacle() = obstacle;
} // updateRoverStatus( Obstacle )

// Updates the odometry information of the rover's status.
void StateMachine::updateRoverStatus( Odometry odometry )
{
    mNewRoverStatus.odometry() = odometry;
} // updateRoverStatus( Odometry )

// Updates the target information of the rover's status.
void StateMachine::updateRoverStatus( TargetList targetList )
{
    Target target1 = targetList.targetList[0];
    Target target2 = targetList.targetList[1];
    mNewRoverStatus.target() = target1;
    mNewRoverStatus.target2() = target2;
} // updateRoverStatus( Target )

// Updates the radio signal strength information of the rover's status.
void StateMachine::updateRoverStatus( RadioSignalStrength radioSignalStrength )
{
    mNewRoverStatus.radio() = radioSignalStrength;
} // updateRoverStatus( RadioSignalStrength )

// Return true if we want to execute a loop in the state machine, false
// otherwise.
bool StateMachine::isRoverReady() const
{
    return mStateChanged || // internal data has changed
           mRover->updateRover( mNewRoverStatus ) || // external data has changed
           mRover->roverStatus().currentState() == NavState::SearchSpinWait || 
           mRover->roverStatus().currentState() == NavState::TurnedToTargetWait || 
           mRover->roverStatus().currentState() == NavState::RepeaterDropWait ||
           mRover->roverStatus().currentState() == NavState::GateSpinWait;
           //The rover is ready if it is waiting. i.e in the states above
           //IF Any one of these options evaluating to true will signal that the rover is ready

} // isRoverReady()

// Publishes the current navigation state to the nav status lcm channel.
void StateMachine::publishNavState() const
{
    NavStatus navStatus;
    navStatus.nav_state_name = stringifyNavState();
    navStatus.completed_wps = mCompletedWaypoints;
    navStatus.total_wps = mTotalWaypoints;
    const string& navStatusChannel = mRoverConfig[ "lcmChannels" ][ "navStatusChannel" ].GetString();
    mLcmObject.publish( navStatusChannel, &navStatus );
} // publishNavState()

// Executes the logic for off. If the rover is turned on, it updates
// the roverStatus. If the course is empty, the rover is done  with
// the course otherwise it will turn to the first waypoing. Else the
// rover is still off.
NavState StateMachine::executeOff()
{
   //If the rover is on 
    if( mRover->roverStatus().autonState().is_auton )
    {
        mCompletedWaypoints = 0;
        mTotalWaypoints = mRover->roverStatus().course().num_waypoints;

        //If there are no more 
        if( !mTotalWaypoints )
        {
            return NavState::Done;
        }
        //If the rover is on and there are still more waypoints go to the turn NavState
        return NavState::Turn;
    }
    mRover->stop(); //Joystick command for rover to stop
    return NavState::Off; //The next navState is off
} // executeOff()

// Executes the logic for the done state. Stops and turns off the
// rover.
NavState StateMachine::executeDone()
{
    mRover->stop();
    return NavState::Done;
} // executeDone()

// Executes the logic for the turning. If the rover is turned off, it
// proceeds to Off. If the rover finishes turning, it drives to the
// next Waypoint. Else the rover keeps turning to the Waypoint.
NavState StateMachine::executeTurn()
{
    if( mRover->roverStatus().path().empty() ) //If the rover's path is empty it is done.
    //Empty as in there are no more waypoints stored in the queue
    {
        return NavState::Done;
    }
    // If we should drop a repeater and have not already, add last
    // point where connection was good to front of path and turn
    if ( isAddRepeaterDropPoint() )
    {
        addRepeaterDropPoint();
        return NavState::RadioRepeaterTurn;
    }

    Odometry& nextPoint = mRover->roverStatus().path().front().odom; //Assigns current Odometry/Bearing of rover to nextPoint
    if( mRover->turn( nextPoint ) ) // If the rover has finished turning.  Current odeometry/bearing is passed
    {
        if (mRover->roverStatus().currentState() == NavState::RadioRepeaterTurn)
        {
            return NavState::RadioRepeaterDrive;
        }
        return NavState::Drive;
    }

    if (mRover->roverStatus().currentState() == NavState::RadioRepeaterTurn)
    {
        return NavState::RadioRepeaterTurn;
    }
    return NavState::Turn;
} // executeTurn()

// Executes the logic for driving. If the rover is turned off, it
// proceeds to Off. If the rover finishes driving, it either starts
// searching for a target (dependent the search parameter of
// the Waypoint) or it turns to the next Waypoint. If the rover
// detects an obstacle and is within the obstacle distance threshold, 
// it goes to turn around it. Else the rover keeps driving to the next Waypoint.
NavState StateMachine::executeDrive()
{
    const Waypoint& nextWaypoint = mRover->roverStatus().path().front();
    double distance = estimateNoneuclid( mRover->roverStatus().odometry(), nextWaypoint.odom );

    // If we should drop a repeater and have not already, add last
    // point where connection was good to front of path and turn
    if ( isAddRepeaterDropPoint() )
    {
        addRepeaterDropPoint();
        return NavState::RadioRepeaterTurn;
    }

    //If an obstacle is detected and the waypoint is not reachable because of it and the obstacle is within the threshold
    if( isObstacleDetected( mRover ) && !isWaypointReachable( distance ) && isObstacleInThreshold( mRover, mRoverConfig ) )
    {
        mObstacleAvoidanceStateMachine->updateObstacleElements( getOptimalAvoidanceAngle(),
                                                                getOptimalAvoidanceDistance() );
        return NavState::TurnAroundObs; //The navstate is the state to turn around the obstacle
    }
    DriveStatus driveStatus = mRover->drive( nextWaypoint.odom );
    if( driveStatus == DriveStatus::Arrived ) //If the rover has arrived at the next waypoint
    {
        if( nextWaypoint.search )
        {
            return NavState::SearchSpin;
        }
        mRover->roverStatus().path().pop_front(); //Removes the waypoint from the front of the queue
        if (mRover->roverStatus().currentState() == NavState::RadioRepeaterDrive)
        {
            return NavState::RepeaterDropWait;
        }
        ++mCompletedWaypoints; //Increment the number of completed waypoints
        return NavState::Turn; //Return turn as the next NavState
    }
    if( driveStatus == DriveStatus::OnCourse ) //If the drive status is on course
    {
        if (mRover->roverStatus().currentState() == NavState::RadioRepeaterDrive)
        {
            return NavState::RadioRepeaterDrive;
        }
        return NavState::Drive;
    }
    // else driveStatus == DriveStatus::OffCourse (must turn to waypoint)
    if (mRover->roverStatus().currentState() == NavState::RadioRepeaterDrive)
    {
        return NavState::RadioRepeaterTurn;
    }

    return NavState::Turn;
} // executeDrive()

// Executes the logic for waiting during a radio repeater drop
// If the rover is done waiting, it continues the original course.
// Else the rover keeps waiting.
NavState StateMachine::executeRepeaterDropWait( )
{

    RepeaterDrop rr_init;
    const string& radioRepeaterInitChannel = mRoverConfig[ "lcmChannels" ][ "repeaterDropInitChannel" ].GetString();
    mLcmObject.publish( radioRepeaterInitChannel, &rr_init );

    if( mRepeaterDropComplete )
    {
        return NavState::Turn;
    }
    return NavState::RepeaterDropWait;
}

// Gets the string representation of a nav state.
string StateMachine::stringifyNavState() const
{
    static const map<NavState, std::string> navStateNames =
        {
            { NavState::Off, "Off" },
            { NavState::Done, "Done" },
            { NavState::Turn, "Turn" },
            { NavState::Drive, "Drive" },
            { NavState::SearchFaceNorth, "Search Face North" },
            { NavState::SearchSpin, "Search Spin" },
            { NavState::SearchSpinWait, "Search Spin Wait" },
            { NavState::ChangeSearchAlg, "Change Search Algorithm" },
            { NavState::SearchTurn, "Search Turn" },
            { NavState::SearchDrive, "Search Drive" },
            { NavState::TurnToTarget, "Turn to Target" },
            { NavState::TurnedToTargetWait, "Turned to Target Wait" },
            { NavState::DriveToTarget, "Drive to Target" },
            { NavState::TurnAroundObs, "Turn Around Obstacle"},
            { NavState::DriveAroundObs, "Drive Around Obstacle" },
            { NavState::SearchTurnAroundObs, "Search Turn Around Obstacle" },
            { NavState::SearchDriveAroundObs, "Search Drive Around Obstacle" },
            { NavState::GateSpin, "Gate Spin" },
            { NavState::GateSpinWait, "Gate Spin Wait" },
            { NavState::GateTurn, "Gate Turn" },
            { NavState::GateDrive, "Gate Drive" },
            { NavState::GateTurnToCentPoint, "Gate Turn to Center Point" },
            { NavState::GateDriveToCentPoint, "Gate Drive to Center Point" },
            { NavState::GateFace, "Gate Face" },
            { NavState::GateShimmy, "Gate Shimmy" },
            { NavState::GateDriveThrough, "Gate Drive Through" },
            { NavState::RadioRepeaterTurn, "Radio Repeater Turn" },
            { NavState::RadioRepeaterDrive, "Radio Repeater Drive" },
            { NavState::RepeaterDropWait, "Radio Repeater Drop" },
            { NavState::Unknown, "Unknown" }
        };

    return navStateNames.at( mRover->roverStatus().currentState() );
} // stringifyNavState()

// Returns the optimal angle to avoid the detected obstacle.
double StateMachine::getOptimalAvoidanceAngle() const
{
    return mRover->roverStatus().obstacle().bearing;
} // optimalAvoidanceAngle()

// Returns the optimal angle to avoid the detected obstacle.
double StateMachine::getOptimalAvoidanceDistance() const
{
    return mRover->roverStatus().obstacle().distance + mRoverConfig[ "navThresholds" ][ "waypointDistance" ].GetDouble();
} // optimalAvoidanceAngle()

//Determines whether or not the next waypoint can be reached
bool StateMachine::isWaypointReachable( double distance )
{
    return isLocationReachable( mRover, mRoverConfig, distance, mRoverConfig["navThresholds"]["waypointDistance"].GetDouble());
} // isWaypointReachable

// If we have not already begun to drop radio repeater
//  (RadioRepeaterTurn or RadioRepeaterDrive)
// We should drop the repeater
// and we haven't already dropped one
bool StateMachine::isAddRepeaterDropPoint() const
{

    return ( mRover->roverStatus().currentState() != NavState::RadioRepeaterTurn &&
             mRover->roverStatus().currentState() != NavState::RadioRepeaterDrive &&
             mRover->isTimeToDropRepeater() &&
             mRepeaterDropComplete == false );
} // isAddRepeaterDropPoint

// Returns whether or not to enter RadioRepeaterTurn state.
void StateMachine::addRepeaterDropPoint()
{
    // TODO: signal drops before first completed waypoint
    // Get last waypoint in path (where signal was good) and set search and gate
    // to false in order to not repeat search
    Waypoint way = (mRover->roverStatus().course().waypoints)[mCompletedWaypoints-1];
    way.search = false;
    way.gate = false;

    mRover->roverStatus().path().push_front(way);
} // addRepeaterDropPoint

// TODOS:
// [drive to target] obstacle and target
// all of code, what to do in cases of both target and obstacle
// thresholds based on state? waypoint vs target
