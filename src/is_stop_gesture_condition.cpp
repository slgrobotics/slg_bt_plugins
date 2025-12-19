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
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        [this](std_msgs::msg::String::SharedPtr msg) {
            // when a gesture is detected, expect steady 2 Hz stream of messages
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

  bool expired = false;
  bool is_stop = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (last_gesture_time_.nanoseconds() == 0 ||
        (node_->now() - last_gesture_time_) > gesture_timeout_)
    {
        // Expire and clear gesture
        last_gesture_.clear();
        last_gesture_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        expired = true;
    }
    else
    {
        gesture = last_gesture_;
    }
  }

  if (!expired) {

    is_stop = (gesture == "STOP"); // gesture can be "STOP", "LIKE", "OK", "YES", "SIX" or empty string

    if(gesture != "") {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[IsStopGesture] tick() gesture: '%s' = %s", gesture.c_str(), is_stop ? "BT:SUCCESS" : "BT:FAILURE");
    }
  } else {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,"[IsStopGesture] tick()  gesture expired - BT:FAILURE");
  }

  /*
   This node acts as a "preemption" trigger.
    - Return SUCCESS when: A stop gesture is actively detected.
      This will trigger the StopGestureSequence, causing CancelControl to run and stop the robot.
    - Return FAILURE when: No stop gesture is seen, or if the data stream ceases (no message received).

    Why: In a ReactiveFallback, returning FAILURE allows the tree to move on to the next priority (Face Detection or Navigation). 
         If it returned SUCCESS during a data dropout, it would inadvertently trigger the stop command. 
  */

  return is_stop
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::FAILURE;
}

}  // namespace slg_bt_plugins

