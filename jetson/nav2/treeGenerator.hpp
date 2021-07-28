#include <string>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <fstream>
#include <sstream>
#include "nodes/vendingMachineNodes.hpp"

class TreeGenerator {
    private:

    std::string xml;
    BT::BehaviorTreeFactory factory;

    void readXML();
    void registerNodes();

    public:

    BT::Tree getTree();


};