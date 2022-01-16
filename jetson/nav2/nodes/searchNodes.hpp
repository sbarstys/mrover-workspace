#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rover.hpp"
#include "searchPatterns/spiralOut.hpp"
#include "searchPatterns/lawnmower.hpp"
#include "searchPatterns/spiralIn.hpp"
namespace searchNodes {
    // Check if the top of the course is a search point
    BT::NodeStatus isSearchPoint();

    // Check if the top of the course is a target approach point
    BT::NodeStatus isTargetApproachPoint();

    // Check if we see target1
    BT::NodeStatus hasTarget1();

    // Check if we see target2
    BT::NodeStatus hasTarget2();

    // Populate post1 with first target information
    BT::NodeStatus populateFirstTarget();

    // Populate post2 with second target information
    BT::NodeStatus populateSecondTarget();

    // Put target at front of course
    BT::NodeStatus putTargetAtFrontOfCourse();

    void registerNodes(BT::BehaviorTreeFactory& factory);

    BT::NodeStatus generateSearchPoints();

}