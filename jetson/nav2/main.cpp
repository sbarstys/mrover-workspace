#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "treeGenerator.hpp";
using namespace rover_msgs;
using namespace std;
using namespace BT;


// TODO LCMS



// Runs the autonomous navigation of the rover.
int main()
{
    TreeGenerator treeGen;

    // reads the tree from xml file, registers the nodes (statically), then generates tree object
    auto tree = treeGen.getTree();

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    BT::NodeStatus status = BT::NodeStatus::SUCCESS;
    while (status == BT::NodeStatus::SUCCESS){
        //lcm processing
        //map processing
        status = tree.tickRoot();
    }


}




