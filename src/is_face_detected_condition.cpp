#include "slg_bt_plugins/is_face_detected_condition.hpp"

namespace slg_bt_plugins
{

IsFaceDetected::IsFaceDetected(const std::string & name,
                const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error(
      "IsFaceDetected: missing 'node' on blackboard");
  }

  RCLCPP_INFO(node_->get_logger(), "[IsFaceDetected] constructor");

  sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/bt/face_detected", 10,
    [this](std_msgs::msg::Bool::SharedPtr msg) {
        face_detected_ = msg->data;
    });
}

BT::NodeStatus IsFaceDetected::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[IsFaceDetected] tick()  face_detected: %s", face_detected_ ? "true" : "false");

  return face_detected_
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::FAILURE;
}

}  // namespace slg_bt_plugins

