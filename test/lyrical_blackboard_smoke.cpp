#include <behaviortree_cpp/bt_factory.h>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <iostream>
#include <action_msgs/srv/cancel_goal.hpp>
#include <thread>
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
  auto server_node = std::make_shared<rclcpp::Node>("cancel_test_server");
  bool request_received = false;
  auto service = server_node->create_service<action_msgs::srv::CancelGoal>(
    "/test_navigation/_action/cancel_goal",
    [&request_received](const std::shared_ptr<action_msgs::srv::CancelGoal::Request> request,
      std::shared_ptr<action_msgs::srv::CancelGoal::Response> response) {
      for (auto byte : request->goal_info.goal_id.uuid) {
        if (byte != 0) throw std::runtime_error("Expected cancel-all UUID");
      }
      if (request->goal_info.stamp.sec != 0 || request->goal_info.stamp.nanosec != 0) {
        throw std::runtime_error("Expected cancel-all timestamp");
      }
      request_received = true;
      response->return_code = action_msgs::srv::CancelGoal::Response::ERROR_NONE;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  auto cancel_tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='CancelTest'>"
    "<CancelNavToPose action_name='/test_navigation' timeout_ms='2000'/>"
    "</BehaviorTree></root>", bb);
  auto result = BT::NodeStatus::RUNNING;
  for (int tick = 0; tick < 1100 && result == BT::NodeStatus::RUNNING; ++tick) {
    executor.spin_some();
    result = cancel_tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (!request_received || result != BT::NodeStatus::SUCCESS) {
    throw std::runtime_error("Cancellation response did not complete without a data pump");
  }
  cancel_tree.haltTree();
  node->shutdown();
  rclcpp::shutdown();
}
