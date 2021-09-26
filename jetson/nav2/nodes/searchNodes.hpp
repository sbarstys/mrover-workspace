#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rover.hpp"

namespace searchNodes {
    BT::NodeStatus isSearchPoint();
    void registerNodes(BT::BehaviorTreeFactory& factory);
    
}