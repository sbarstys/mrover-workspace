#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rover.hpp"

namespace searchNodes {
    // is the top of the course a search point?
    BT::NodeStatus isSearchPoint();

    // do we see a target?
    BT::NodeStatus hasTarget();

    // is the top of the course a target approach point?
    BT::NodeStatus isTargetApproachPoint();

    // 
    BT::NodeStatus populateFirstTarget();

    // 
    BT::NodeStatus populateSecondTarget();

    BT::NodeStatus putTargetAtFrontOfCourse();

    void registerNodes(BT::BehaviorTreeFactory& factory);
    
}