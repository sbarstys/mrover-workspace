#include "utilityNodes.hpp"
#include "rover.hpp"
namespace utilNodes{

    BT::NodeStatus isOff(){

        if ( !gRover->autonState().is_auton ){
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus atCurrentWaypoint(){
        if ( estimateNoneuclid(gRover->odometry(),
                               gRover->roverStatus().course().front().odom) < 1.0 ) { // TODO determine distance threshold
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus spinGimbal(){
        // spins gimbal
        // uses retry decoratory to spin the gimbal upto N times
        // Fails: if at the end of the gimbal angles trajectory
        // Succeeds: otherwise
        bool at_desired_angle = gRover->sendGimbalSetpoint( gRover->gimbalAngles()[gRover->gimbalIndex()] );

        if ( gRover->gimbalIndex() == (int)gRover->gimbalAngles().size()-1 ){
            // at end of spin cycle - haven't seen target, give up
            gRover->gimbalIndex() = 0;
            return BT::NodeStatus::FAILURE;
        }
        else{
            gRover->gimbalIndex()++;
            return BT::NodeStatus::SUCCESS;
        }
    }

    BT::NodeStatus resetGimbalIndex(){
        // resets the gimbal index to 0 such that the trajectory of the
        // gimbal spin is reset
        // TODO may need to point the gimbal forward again after the spin manuever is done
        gRover->gimbalIndex() = 0;
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus popCourse(){
        gRover->roverStatus().course().pop_front();
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus isDestinationPoint(){
        if ( gRover->roverStatus().course().front().type == "destination" ){
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;

    }

    BT::NodeStatus isARTagLeg(){
        // gate = false
        // search = true
        if ( gRover->roverStatus().course().front().gate == false &&
             gRover->roverStatus().course().front().search == true ){

            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus clearCourse(){
        gRover->roverStatus().course().clear();
        return BT::NodeStatus::SUCCESS;

    }

    BT::NodeStatus turnOff(){
        gRover->roverStatus().autonState().is_auton = false;
        return BT::NodeStatus::SUCCESS;
    }

    void registerNodes(BT::BehaviorTreeFactory& factory){
        factory.registerSimpleAction( "isOff", std::bind(isOff) );
        factory.registerSimpleAction( "atCurrentWaypoint", std::bind(atCurrentWaypoint) );
        factory.registerSimpleAction( "spinGimbal", std::bind(spinGimbal) );
        factory.registerSimpleAction( "resetGimbalIndex", std::bind(resetGimbalIndex) );
        factory.registerSimpleAction( "popCourse", std::bind(popCourse) );
        factory.registerSimpleAction( "isDestinationPoint", std::bind(isDestinationPoint) );
        factory.registerSimpleAction( "isARTagLeg", std::bind(isARTagLeg) );
        factory.registerSimpleAction( "clearCourse", std::bind(clearCourse) );
        factory.registerSimpleAction( "turnOff", std::bind(turnOff) );


    }
}

