#include "slg_bt_plugins/is_blackboard_string_equal_condition.hpp"

namespace slg_bt_plugins
{

IsBlackboardStringEqual::IsBlackboardStringEqual(const std::string & name,
                                                 const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("IsBlackboardStringEqual: missing 'node' on blackboard");
  }
  RCLCPP_INFO(node_->get_logger(), "[IsBlackboardStringEqual] constructor");
}

BT::NodeStatus IsBlackboardStringEqual::tick()
{
  std::string key_name;
  if (!getInput("key", key_name) || key_name.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[IsBlackboardStringEqual] tick() - missing/empty 'key' (expected blackboard key name) - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  std::string compare_to;
  if (!getInput("compare_to_string", compare_to)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[IsBlackboardStringEqual] tick() - missing 'compare_to_string' - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  std::string current;
  if (!config().blackboard->get(key_name, current)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[IsBlackboardStringEqual] tick() - blackboard missing key '%s' - BT:FAILURE",
      key_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const bool equal = (current == compare_to);

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[IsBlackboardStringEqual] '%s'='%s' compare_to='%s' -> %s",
    key_name.c_str(), current.c_str(), compare_to.c_str(),
    equal ? "BT:SUCCESS" : "BT:FAILURE");

  return equal ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace slg_bt_plugins
