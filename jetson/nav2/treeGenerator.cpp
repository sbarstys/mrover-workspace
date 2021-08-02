#include "treeGenerator.hpp"

using std::ifstream;
using std::ostringstream;


void TreeGenerator::readXML(){
    const std::string path = "vendingMachine.xml";
    ifstream input_file(path);
    if (!input_file.is_open()){
        std::cout << "you're code sucks" << std::endl;
    }
    xml = std::string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
    //TODO: use raw strings
    xml = "(" + xml + ")";
}

void TreeGenerator::registerNodes(){
    //TODO: regiser nodes (for actual tree)


    //vending machine registration TODO: delete this and replace with real tree
    using namespace VendingMachineNodes;

    fillDrinkTable();

    factory.registerSimpleCondition("CheckOverallDrinkAvail", std::bind(CheckOverallDrinkAvail));
    factory.registerSimpleCondition("CheckMachineStatus", std::bind(CheckMachineStatus));
    factory.registerSimpleAction("PrintOptions", std::bind(PrintOptions));
    //dont use the register node type when we do it for real (no need for blackboard)
    factory.registerNodeType<Prompt>("Prompt");
    factory.registerNodeType<DispenseDrink>("DispenseDrink");
    factory.registerNodeType<CheckSpace>("CheckSpace");

}

BT::Tree TreeGenerator::getTree(){
    readXML();
    registerNodes();
    auto tree = factory.createTreeFromText(xml);
    return tree;
}