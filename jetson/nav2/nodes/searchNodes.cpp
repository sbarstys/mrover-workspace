#include "searchNodes.hpp"
#include "utilities.hpp"
#include "rover_msgs/Odometry.hpp"

namespace searchNodes{
    BT::NodeStatus isSearchPoint(){
        if (gRover->roverStatus().course().peekTop().type == "search"){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
        
    }

    BT::NodeStatus isTargetApproachPoint(){
        if (gRover->roverStatus().course().peekTop().type == "targetApproach"){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus hasTarget(){
        if (gRover->roverStatus().target().distance > 0.0 || gRover->roverStatus().target2().distance > 0.0){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus populateFirstTarget() {
        Odometry gateLoc = createOdom(gRover->roverStatus().odometry(), gRover->roverStatus().target().bearing, gRover->roverStatus().target().distance, gRover);
        gRover->roverStatus().post1().id = gRover->roverStatus().target().id;
        gRover->roverStatus().post1().location = gateLoc;
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus populateSecondTarget() {
        Odometry gateLoc = createOdom(gRover->roverStatus().odometry(), gRover->roverStatus().target2().bearing, gRover->roverStatus().target2().distance, gRover);
        gRover->roverStatus().post2().id = gRover->roverStatus().target2().id;
        gRover->roverStatus().post2().location = gateLoc;
        return BT::NodeStatus::SUCCESS;
    }


    BT::NodeStatus putTargetAtFrontOfCourse(){
        Odometry locationToGo = gRover->roverStatus().post1().location;
        Waypoint waypoint;
        waypoint.odom = locationToGo;
        waypoint.type = "targetApproach";
        gRover->roverStatus().course().pushFront(waypoint);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus generateSearchPoints(){
        /*
        TODO fill this in lol
        */
        return BT::NodeStatus::SUCCESS;
    }

    
    void registerNodes(BT::BehaviorTreeFactory& factory){
        factory.registerSimpleCondition("isSearchPoint", std::bind(isSearchPoint));
        factory.registerSimpleCondition("isTargetApproachPoint", std::bind(isTargetApproachPoint));
        factory.registerSimpleCondition("hasTarget", std::bind(hasTarget));
        factory.registerSimpleCondition("populateFirstTarget", std::bind(populateFirstTarget));
        factory.registerSimpleCondition("populateSecondTarget", std::bind(populateSecondTarget));
        factory.registerSimpleCondition("putTargetAtFrontOfCourse", std::bind(putTargetAtFrontOfCourse));
    }
}