#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace utilNodes {
    void registerNodes(BT::BehaviorTreeFactory& factory);

    BT::NodeStatus isOff();

    BT::NodeStatus atCurrentWaypoint();

    // TODO we have to flesh this out much more - where do we use? Gate nodes, search nodes
    BT::NodeStatus spinGimbal();

    BT::NodeStatus resetGimbalIndex();

    BT::NodeStatus popCourse();

    BT::NodeStatus isDestinationPoint();

    BT::NodeStatus isARTagLeg();

    BT::NodeStatus clearCourse();

    BT::NodeStatus turnOff();

}