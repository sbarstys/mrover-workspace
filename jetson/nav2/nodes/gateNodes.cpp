#include "gateNodes.hpp"
#include "rover.hpp"

namespace gateNodes{

    // TODO: not done
    BT::NodeStatus isFirstGatePostLocKnown(){
        if (gRover->roverStatus().FirstGatePostFound()){
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    void registerNodes(BT::BehaviorTreeFactory& factory){
        factory.registerSimpleAction("isFirstGatePostLocKnown", std::bind(isFirstGatePostLocKnown));
    }

}