#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace gateNodes {
    void registerNodes(BT::BehaviorTreeFactory& factory);

    // Know First Gate Post Location?
    BT::NodeStatus isFirstGatePostLocKnown();

    // Generate Gate Traversal Path
    BT::NodeStatus genGateTraversalPath();

    // Gate Traversal Point?
    BT::NodeStatus isGateTraversalPoint();

    // No more Gate Traversal Waypoint left?
    BT::NodeStatus noGateTraversalWaypoints();

    // Traversal in wrong direction?
    BT::NodeStatus didTraverseWrongDirection();

    // Regenerate gate traversal waypoints
    BT::NodeStatus regenGateTraversalWaypoints();

    // More Gate traversal waypoints left?
    BT::NodeStatus hasGateTraversalPoints();

    // note gate post 1 found, gen search points
    BT::NodeStatus genSecondPostSearchPattern();


}
