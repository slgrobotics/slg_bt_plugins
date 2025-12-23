#include "slg_bt_plugins/is_false_condition.hpp"

namespace slg_bt_plugins
{

IsFalse::IsFalse(const std::string & name,
                 const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("IsFalse: missing 'node' on blackboard");
  }
  RCLCPP_INFO(node_->get_logger(), "[IsFalse] constructor");
}

BT::NodeStatus IsFalse::tick()
{
  bool val = false;
  if (!getInput("key", val)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[IsFalse] tick() - missing 'key' input (expected bool) - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  const bool is_false = !val;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "[IsFalse] tick() key=%s -> %s",
    val ? "true" : "false",
    is_false ? "BT:SUCCESS" : "BT:FAILURE");

  return is_false ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace slg_bt_plugins
