#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rover.hpp"
#include "treeGenerator.hpp";
using namespace rover_msgs;
using namespace std;
using namespace BT;


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
        mStatus->updateRoverStatus( *autonState );
    }

    // Sends the course lcm message to the state machine.
    void destinations(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const Destinations* destinations
        )
    {
        mStatus->updateRoverStatus( *destinations );
    }

    // Sends the obstacle lcm message to the state machine.
    void obstacle(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const Obstacle* obstacle
        )
    {
        mStatus->updateRoverStatus( *obstacle );
    }

    // Sends the odometry lcm message to the state machine.
    void odometry(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const Odometry* odometry
        )
    {
        mStatus->updateRoverStatus( *odometry );
    }

    // Sends the target lcm message to the state machine.
    void targetList(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const TargetList* targetListIn
        )
    {
        mStatus->updateRoverStatus( *targetListIn );
    }

    // Sends the radio lcm message to the state machine.
    void radioSignalStrength(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const RadioSignalStrength* signalIn
        )
    {
        mStatus->updateRoverStatus( *signalIn );
    }

    // Updates Radio Repeater bool in state machine.
    void repeaterDropComplete(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const RepeaterDrop* completeIn
        )
    {
        mStatus->updateRepeaterComplete( );
    }

private:
    // The state machine to send the lcm messages to.
    Rover::RoverStatus mStatus;
};



// Runs the autonomous navigation of the rover.
int main()
{
    //TODO: get the rover object from a seperate function (uses rover config)
    gRover = new Rover();

    //initialize lcms
    lcm::LCM lcmObject;
    if( !lcmObject.good() )
    {
        cerr << "Error: cannot create LCM\n";
        return 1;
    }

    StateMachine roverStateMachine( lcmObject );
    LcmHandlers lcmHandlers( &roverStateMachine );

    lcmObject.subscribe( "/auton", &LcmHandlers::autonState, &lcmHandlers );
    lcmObject.subscribe( "/course", &LcmHandlers::course, &lcmHandlers );
    lcmObject.subscribe( "/obstacle", &LcmHandlers::obstacle, &lcmHandlers );
    lcmObject.subscribe( "/odometry", &LcmHandlers::odometry, &lcmHandlers );
    lcmObject.subscribe( "/radio", &LcmHandlers::radioSignalStrength, &lcmHandlers );
    lcmObject.subscribe( "/rr_drop_complete", &LcmHandlers::repeaterDropComplete, &lcmHandlers );
    lcmObject.subscribe( "/target_list", &LcmHandlers::targetList, &lcmHandlers );




    TreeGenerator treeGen;

    // reads the tree from xml file, registers the nodes (statically), then generates tree object
    auto tree = treeGen.getTree();

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.s
    BT::NodeStatus status = BT::NodeStatus::SUCCESS;
    while (status == BT::NodeStatus::SUCCESS && (lcmObject.handle() == 0)){
        //TODO: map processing
        status = tree.tickRoot();
    }


}




