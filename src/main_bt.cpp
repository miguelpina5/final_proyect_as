#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/xml_parsing.h>
#include "final_proyect_as/DetectPersonNode.hpp"
#include "final_proyect_as/FinishNode.hpp"
#include "final_proyect_as/RequestPlayersNode.hpp"

using namespace BT;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_node");

    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;
    factory.registerFromPlugin(loader.getOSName("detectPerson_bt_node"));
    factory.registerFromPlugin(loader.getOSName("finish_bt_node"));
    factory.registerFromPlugin(loader.getOSName("requestPlayer_bt_node"));

    std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_nav");
    std::string xml_file = pkgpath + "/behavior_tree_xml/navigate.xml";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

    rclcpp::Rate rate(10);

    bool finish = false;
    while (rclcpp::ok() && !finish) {
      rclcpp::spin_some(node);
      finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
      rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
