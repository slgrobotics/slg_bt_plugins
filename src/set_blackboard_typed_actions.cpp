#include "slg_bt_plugins/set_blackboard_typed_actions.hpp"

namespace slg_bt_plugins
{

// -------------------- SetBlackboardInt --------------------

SetBlackboardInt::SetBlackboardInt(const std::string & name,
                                   const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("SetBlackboardInt: missing 'node' on blackboard");
  }
  RCLCPP_INFO(node_->get_logger(), "[SetBlackboardInt] constructor");
}

BT::NodeStatus SetBlackboardInt::tick()
{
  std::string output_key;
  if (!getInput("output_key", output_key) || output_key.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[SetBlackboardInt] tick() - missing/empty 'output_key' - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  int value = 0;
  if (!getInput("value", value)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[SetBlackboardInt] tick() - missing 'value' input (expected int) - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  config().blackboard->set(output_key, value);

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[SetBlackboardInt] set '%s' = %d -> BT:SUCCESS", output_key.c_str(), value);

  return BT::NodeStatus::SUCCESS;
}

// -------------------- SetBlackboardDouble --------------------

SetBlackboardDouble::SetBlackboardDouble(const std::string & name,
                                         const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("SetBlackboardDouble: missing 'node' on blackboard");
  }
  RCLCPP_INFO(node_->get_logger(), "[SetBlackboardDouble] constructor");
}

BT::NodeStatus SetBlackboardDouble::tick()
{
  std::string output_key;
  if (!getInput("output_key", output_key) || output_key.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[SetBlackboardDouble] tick() - missing/empty 'output_key' - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  double value = 0.0;
  if (!getInput("value", value)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[SetBlackboardDouble] tick() - missing 'value' input (expected double) - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  config().blackboard->set(output_key, value);

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[SetBlackboardDouble] set '%s' = %.6f -> BT:SUCCESS", output_key.c_str(), value);

  return BT::NodeStatus::SUCCESS;
}

// -------------------- SetBlackboardString --------------------

SetBlackboardString::SetBlackboardString(const std::string & name,
                                         const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("SetBlackboardString: missing 'node' on blackboard");
  }
  RCLCPP_INFO(node_->get_logger(), "[SetBlackboardString] constructor");
}

BT::NodeStatus SetBlackboardString::tick()
{
  std::string output_key;
  if (!getInput("output_key", output_key) || output_key.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[SetBlackboardString] tick() - missing/empty 'output_key' - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  std::string value;
  if (!getInput("value", value)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[SetBlackboardString] tick() - missing 'value' input (expected string) - BT:FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  config().blackboard->set(output_key, value);

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[SetBlackboardString] set '%s' = '%s' -> BT:SUCCESS", output_key.c_str(), value.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace slg_bt_plugins
