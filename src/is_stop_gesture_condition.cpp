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

  // Note: the "Callback Group" solution didn't work for me as expected.

  sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/bt/gesture_command",
        rclcpp::QoS(1).best_effort(),
        [this](std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_WARN(node_->get_logger(), "[IsStopGesture] sub received gesture: '%s'", msg->data.c_str());
            std::lock_guard<std::mutex> lock(mutex_);
            last_gesture_ = msg->data;
            last_gesture_time_ = node_->now();
        }
    );
}

// Nav2 ticks BTs at ~10–30 Hz.

BT::NodeStatus IsStopGesture::tick()
{
  // The subscription happens, but messages are queued until the node is spun.
  // So, we have to manually process any waiting messages for the BT node:
  rclcpp::spin_some(node_->get_node_base_interface());  // executes any pending callbacks on that node
  
  std::string gesture;
  rclcpp::Time stamp;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    gesture = last_gesture_;
    stamp = last_gesture_time_;
  }

  const auto now = node_->now();

  // Expire gesture
  if (stamp.nanoseconds() == 0 || (now - stamp) > gesture_timeout_)
  {
    RCLCPP_INFO(node_->get_logger(), "[IsStopGesture] gesture expired");

    std::lock_guard<std::mutex> lock(mutex_);
    last_gesture_.clear();
    last_gesture_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

    return BT::NodeStatus::FAILURE;
  }

  const bool is_stop = (gesture == "STOP");

  if(gesture != "") {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
       "[IsStopGesture] tick() gesture: '%s' = %s", gesture.c_str(), is_stop ? "BT:SUCCESS" : "BT:FAILURE");
  }

  return is_stop
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::FAILURE;
}

}  // namespace slg_bt_plugins

