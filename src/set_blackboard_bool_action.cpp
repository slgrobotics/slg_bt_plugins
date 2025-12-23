#include "slg_bt_plugins/set_blackboard_bool_action.hpp"

namespace slg_bt_plugins
{

SetBlackboardBool::SetBlackboardBool(const std::string & name,
                                     const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("SetBlackboardBool: missing 'node' on blackboard");
  }

  RCLCPP_INFO(node_->get_logger(), "[SetBlackboardBool] constructor");
}

BT::NodeStatus SetBlackboardBool::tick()
{
  std::string output_key;
  if (!getInput("output_key", output_key) || output_key.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[SetBlackboardBool] tick() - missing/empty 'output_key' - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  bool value = false;
  if (!getInput("value", value)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[SetBlackboardBool] tick() - missing 'value' input (expected bool) - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  // Write to the BT blackboard
  config().blackboard->set(output_key, value);

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "[SetBlackboardBool] set '%s' = %s -> BT:SUCCESS",
    output_key.c_str(), value ? "true" : "false");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace slg_bt_plugins
