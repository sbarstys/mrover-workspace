#include "treeGenerator.hpp"

using std::ifstream;
using std::ostringstream;


void TreeGenerator::readXML(){
    ifstream xmlFile;
    string xmlPath = getenv("MROVER_CONFIG");
    xmlPath += "/config_nav/vendingMachine.xml";
    xmlFile.open( xmlPath );
    if (!xmlFile.is_open()){
        std::cout << "your code sucks" << std::endl;
    }
    xml = std::string((std::istreambuf_iterator<char>(xmlFile)), std::istreambuf_iterator<char>());
    //TODO: use raw strings
    xml = "(" + xml + ")";
    std::cout << xml << std::endl;
    xml.erase(std::remove(xml.begin(), xml.end(), '\n'), xml.end());
    xml.erase(std::remove(xml.begin(), xml.end(), '\t'), xml.end());
    std::cout << endl;
    std::cout << xml << std::endl;
}

void TreeGenerator::registerNodes(){
    //TODO: regiser nodes (for actual tree)
    utilNodes::registerNodes(factory);
    gateNodes::registerNodes(factory);
    searchNodes::registerNodes(factory);


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
    
    string xmlPath = getenv("MROVER_CONFIG");
    xmlPath += "/config_nav/masterTree.xml";
    std::cout << "hello world" << std::endl;
    auto tree = factory.createTreeFromFile(xmlPath);
    
    std::cout << "LOL" << std::endl;
    //auto tree = factory.createTreeFromText(xml);
    return tree;
}