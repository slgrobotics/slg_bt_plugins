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

#ifdef USE_RCLCPP_SUBSCRIPTIONS
  // Note: the "Callback Group" solution didn't work for me as expected.

  sub_ = node_->create_subscription<sensor_msgs::msg::Illuminance>(
        "/bt/face_gesture_detect",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        // Note: the Lambda callback will be called when BT node is spun and only then the messages will be received, including the queued stale ones
        [this](sensor_msgs::msg::Illuminance::SharedPtr msg) {
            // when a gesture is detected, expect steady 2 Hz stream of messages
            RCLCPP_WARN(node_->get_logger(), "[IsStopGesture] sub received gesture: '%s'", msg->header.frame_id.c_str());
            std::lock_guard<std::mutex> lock(mutex_);
            last_gesture_ = msg->header.frame_id;   // using frame_id field for gesture string
            last_gesture_time_ = rclcpp::Time(msg->header.stamp, node_->get_clock()->get_clock_type()); // time when the message was sent
        }
    );
#endif // USE_RCLCPP_SUBSCRIPTIONS
}

// Nav2 ticks BTs at ~10–30 Hz.

BT::NodeStatus IsStopGesture::tick()
{
  bool is_stop = false;

#ifdef USE_RCLCPP_SUBSCRIPTIONS
  // The subscription happens, but messages are queued until the node is spun.
  // So, we have to manually process any waiting messages for the BT node:
  rclcpp::spin_some(node_->get_node_base_interface());  // executes any pending callbacks on that node
  
  std::string gesture;

  bool expired = false;

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
#else // USE_RCLCPP_SUBSCRIPTIONS

  getInput("is_stop_gesture", is_stop);  // get the gesture from the blackboard

#endif // USE_RCLCPP_SUBSCRIPTIONS

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

