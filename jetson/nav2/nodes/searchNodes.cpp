#include "searchNodes.hpp"

namespace searchNodes{
    BT::NodeStatus isSearchPoint(){
        if (gRover->roverStatus().course().peekTop().type == "search"){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
    void registerNodes(BT::BehaviorTreeFactory& factory){
         factory.registerSimpleAction("isSearhPoint", std::bind(isSearchPoint));
    }
}