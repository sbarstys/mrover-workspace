#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "rover.hpp"
#include "simplePathFollower.hpp"
#include "rover_msgs/Obstacle.hpp"
#include <fstream>
using namespace rover_msgs;
using namespace std;


// TODO, we need to create the variables and udpate functions in RoverStatus (preivously in StateMachine)
// This class handles all incoming LCM messages for the autonomous
// navigation of the rover.
class LcmHandlers
{
public:
    // Constructs an LcmHandler with the given status to work
    // with.
    LcmHandlers( Rover::RoverStatus* status )
        : mStatus( status )
    {}

    // Sends the auton state lcm message to the state machine.
    void autonState(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const AutonState* autonState
        )
    {
        //TODO: fill this in with an actual implementation that would trigger the start of the behavior tree
        //mStatus->updateRoverStatus( *autonState );
    }

    // Sends the Destinations lcm message to the state machine.
    void destinations(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const Destinations* dests
        )
    {
        mStatus->destinations() = *dests;
    }

    // Sends the obstacle lcm message to the state machine.
    void obstacle(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const Obstacle* obstacle
        )
    {
        //TODO: set oobstacles in rover state
        //mStatus->updateRoverStatus( *obstacle );
    }

    // Sends the odometry lcm message to the state machine.
    void odometry(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const Odometry* odometry
        )
    {
        mStatus->odometry() = *odometry;
    }

    // Sends the target lcm message to the state machine.
    void targetList(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const TargetList* targetListIn
        )
    {
        //TODO: update for rover status based config
        //mStatus->updateRoverStatus( *targetListIn );
    }

    // Sends the radio lcm message to the state machine.
    void radioSignalStrength(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const RadioSignalStrength* signalIn
        )
    {
        //TODO: update for rover status based config
        //mStatus->updateRoverStatus( *signalIn );
    }

    // Updates Radio Repeater bool in state machine.
    void repeaterDropComplete(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const RepeaterDrop* completeIn
        )
    {
        //TODO: updatte for rover status based config
        //mStatus->updateRepeaterComplete( );
    }

private:
    // The state machine to send the lcm messages to.
    Rover::RoverStatus* mStatus;
};



// Runs the autonomous navigation of the rover.
int main()
{
    lcm::LCM lcmObject;
    if( !lcmObject.good() )
    {
        cerr << "Error: cannot create LCM\n";
        return 1;
    }
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_nav/config.json";
    configFile.open( configPath );
    string config = "";
    string setting;
    while( configFile >> setting )
    {
        config += setting;
    }
    configFile.close();
    rapidjson::Document roverConfig;
    roverConfig.Parse( config.c_str() );
    gRover = new Rover( roverConfig, lcmObject );

    //initialize lcms
    

    LcmHandlers lcmHandlers( &(gRover->roverStatus()) );

    lcmObject.subscribe( "/auton", &LcmHandlers::autonState, &lcmHandlers );
    lcmObject.subscribe( "/course", &LcmHandlers::destinations, &lcmHandlers );
    lcmObject.subscribe( "/obstacle", &LcmHandlers::obstacle, &lcmHandlers );
    lcmObject.subscribe( "/odometry", &LcmHandlers::odometry, &lcmHandlers );
    lcmObject.subscribe( "/radio", &LcmHandlers::radioSignalStrength, &lcmHandlers );
    lcmObject.subscribe( "/rr_drop_complete", &LcmHandlers::repeaterDropComplete, &lcmHandlers );
    lcmObject.subscribe( "/target_list", &LcmHandlers::targetList, &lcmHandlers );

    Odometry newPoint = gRover->roverStatus().odometry();
    newPoint.longitude_min += 1;
    SimplePathFollower stateMachine;
    std::vector<Odometry> path;

    stateMachine.followPath(path);
     

    return 0;
}