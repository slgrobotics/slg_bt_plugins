#include "slg_bt_plugins/is_stop_gesture_condition.hpp"

namespace slg_bt_plugins
{

IsStopGesture::IsStopGesture(const std::string & name,
                const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error(
        "IsStopGesture: missing 'node' on blackboard");
  }

  RCLCPP_INFO(node_->get_logger(), "[IsStopGesture] constructor");

  // See https://chatgpt.com/s/t_69435fadacf88191989652b62658ac5d
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/bt/gesture_command",
        rclcpp::QoS(1).best_effort(),
        [this](std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_WARN(node_->get_logger(), "[IsStopGesture] sub received gesture: '%s'", msg->data.c_str());
            std::lock_guard<std::mutex> lock(mutex_);
            last_gesture_ = msg->data;
        },
        options
    );

  sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/bt/gesture_command", 10,
    [this](std_msgs::msg::String::SharedPtr msg) {
        last_gesture_ = msg->data;
    });
}

BT::NodeStatus IsStopGesture::tick()
{
  std::string gesture;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    gesture = last_gesture_;
  }

  bool ret = gesture == "STOP";

  RCLCPP_INFO(node_->get_logger(), "[IsStopGesture] tick() gesture: '%s' = %s", gesture.c_str(), ret ? "BT:SUCCESS" : "BT:FAILURE");

  return ret
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::FAILURE;
}

}  // namespace slg_bt_plugins

