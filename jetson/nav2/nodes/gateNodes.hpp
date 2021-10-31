#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "searchPatterns/diamondGateSearch.hpp"

namespace gateNodes {
    void registerNodes(BT::BehaviorTreeFactory& factory);

    // Know First Gate Post Location?
    BT::NodeStatus isFirstGatePostLocKnown();

    // helper
    bool genGateTraversalPathHelper();

    // Generate Gate Traversal Path
    BT::NodeStatus genGateTraversalPath();

    // Gate Traversal Point?
    BT::NodeStatus isGateTraversalPoint();

    // More Gate traversal waypoints left?
    BT::NodeStatus hasGateTraversalPoints();

    //
    BT::NodeStatus verifyGateTraversal();

    // note gate post 1 found, gen search points
    BT::NodeStatus genSecondPostSearchPattern();


}
