#include <behaviortree_cpp/bt_factory.h>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <iostream>
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const std::string plugin_path = argv[1];
  const std::string tree_path = argv[2];
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2::LifecycleNode>("slg_bt_migration_test");
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(plugin_path);
  auto bb = BT::Blackboard::create();
  bb->set<nav2::LifecycleNode::SharedPtr>("node", node);
  for (const auto &entry : factory.manifests()) {
    if (factory.builtinNodes().count(entry.first)) continue;
    const auto attr = entry.first == "FgsTopicToBlackboard" ? " topic_name='/test/perception'" : "";
    auto tree = factory.createTreeFromText("<root BTCPP_format='4'><BehaviorTree ID='Test'><" + entry.first + attr + "/></BehaviorTree></root>", bb);
    std::cout << "Constructed " << entry.first << '\n';
  }
  auto tree = factory.createTreeFromFile(tree_path, bb);
  tree.tickOnce();
  if (bb->get<bool>("is_face_detected") || !bb->get<std::string>("gesture").empty()) return 1;
  tree.haltTree();
  node->shutdown();
  rclcpp::shutdown();
}
