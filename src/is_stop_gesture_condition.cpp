#include "slg_bt_plugins/is_stop_gesture_condition.hpp"

namespace slg_bt_plugins
{

IsStopGesture::IsStopGesture(const std::string & name,
                const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  RCLCPP_INFO(node_->get_logger(), "[IsStopGesture] constructor");

  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/bt/gesture_command", 10,
    std::bind(&IsStopGesture::gestureCallback, this, std::placeholders::_1));
}

BT::PortsList IsStopGesture::providedPorts()
{
  return {};
}

BT::NodeStatus IsStopGesture::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[IsStopGesture] tick()");

  if (last_gesture_ == "STOP") {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

void IsStopGesture::gestureCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "[IsStopGesture] gestureCallback() received: %s", msg->data.c_str());

  last_gesture_ = msg->data;
}

}  // namespace slg_bt_plugins

