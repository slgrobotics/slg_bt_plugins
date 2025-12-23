#include "slg_bt_plugins/is_true_condition.hpp"

namespace slg_bt_plugins
{

IsTrue::IsTrue(const std::string & name,
               const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("IsTrue: missing 'node' on blackboard");
  }

  RCLCPP_INFO(node_->get_logger(), "[IsTrue] constructor");
}

BT::NodeStatus IsTrue::tick()
{
  bool val = false;
  if (!getInput("key", val)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[IsTrue] tick() - missing 'key' input (expected bool) - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "[IsTrue] tick() key=%s -> %s",
    val ? "true" : "false",
    val ? "BT:SUCCESS" : "BT:FAILURE");

  return val ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace slg_bt_plugins
