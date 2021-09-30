#include "searchNodes.hpp"

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
        
    }

    BT::NodeStatus populateFirstTarget() {

    }

    BT::NodeStatus populateSecondTarget() {

    }

    
    void registerNodes(BT::BehaviorTreeFactory& factory){
        factory.registerSimpleCondition("isSearchPoint", std::bind(isSearchPoint));
        factory.registerSimpleCondition("isTargetApproachPoint", std::bind(isTargetApproachPoint));
        factory.registerSimpleCondition("hasTarget", std::bind(hasTarget));
        factory.registerSimpleCondition("populateFirstTarget", std::bind(populateFirstTarget));
        factory.registerSimpleCondition("populateSecondTarget", std::bind(populateSecondTarget));

    }
}