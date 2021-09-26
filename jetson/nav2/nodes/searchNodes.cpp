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


    void registerNodes(BT::BehaviorTreeFactory& factory){
         factory.registerSimpleCondition("isSearhPoint", std::bind(isSearchPoint));
         factory.registerSimpleCondition("isTargetApproachPoint", std::bind(isSearchPoint));
    }
}