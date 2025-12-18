#include "slg_bt_plugins/is_stop_gesture_condition.hpp"

namespace slg_bt_plugins
{

IsStopGesture::IsStopGesture(const std::string & name,
                const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

//   if (!config.blackboard->get("node", node_)) {
//     throw std::runtime_error(
//       "IsStopGesture: missing 'node' on blackboard");
//   }

  RCLCPP_INFO(node_->get_logger(), "[IsStopGesture] constructor");

  sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/bt/gesture_command", 10,
    [this](std_msgs::msg::String::SharedPtr msg) {
        last_gesture_ = msg->data;
    });
}

BT::NodeStatus IsStopGesture::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[IsStopGesture] tick() last_gesture_: %s", last_gesture_.c_str());

  return last_gesture_ == "STOP"
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::FAILURE;
}

}  // namespace slg_bt_plugins

