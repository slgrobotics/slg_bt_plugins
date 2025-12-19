#include "slg_bt_plugins/fgs_to_bt_node.hpp"

namespace slg_bt_plugins
{

using std::placeholders::_1;

FgsTopicToBlackboard::FgsTopicToBlackboard(const std::string & name,
                const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error(
        "FgsTopicToBlackboard: missing 'node' on blackboard");
  }

  getInput("topic_name", topic_name_);

  RCLCPP_INFO(node_->get_logger(), "[FgsTopicToBlackboard] constructor, topic: '%s'", topic_name_.c_str());

  // Note: the "Callback Group" solution didn't work for me as expected.

  // or rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile()
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  sub_ = node_->create_subscription<sensor_msgs::msg::Illuminance>(
    topic_name_, qos, std::bind(&FgsTopicToBlackboard::callbackFgsPerceptionAdapterMessage, this, _1));
}

void
FgsTopicToBlackboard::callbackFgsPerceptionAdapterMessage(const sensor_msgs::msg::Illuminance::SharedPtr msg)
{
    // Note: the Lambda callback will be called when BT node is spun and only then the messages will be received,
    // including the queued stale ones
    // when a face or gesture is detected, expect steady 10 Hz stream of messages
    RCLCPP_WARN(node_->get_logger(), "[FgsTopicToBlackboard] sub received gesture: '%s'", msg->header.frame_id.c_str());

    std::lock_guard<std::mutex> lock(mutex_);
    is_face_detected_ = msg->illuminance > 0.5;  // using illuminance field as boolean
    face_yaw_error_ = msg->variance;             // using variance field for yaw error
    last_gesture_ = msg->header.frame_id;        // using frame_id field for gesture string
    last_message_time_ = rclcpp::Time(msg->header.stamp, node_->get_clock()->get_clock_type()); // time when the message was sent
}

// Nav2 ticks BTs at ~10–30 Hz.

BT::NodeStatus FgsTopicToBlackboard::tick()
{
  // The subscription happens, but messages are queued until the node is spun.
  // So, we have to manually process any waiting messages for the BT node:
  rclcpp::spin_some(node_->get_node_base_interface());  // executes any pending callbacks on that node
  
  std::string gesture;

  bool expired = false;
  bool is_stop = false;
  bool is_face_detected = false;
  float face_yaw_error = 0.0;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (last_message_time_.nanoseconds() == 0 ||
        (node_->now() - last_message_time_) > message_timeout_)
    {
        // Expire and clear gesture
        last_gesture_.clear();
        last_message_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        expired = true;
    }
    else
    {
        is_face_detected = is_face_detected_;
        face_yaw_error = face_yaw_error_;
        gesture = last_gesture_;
    }
  }

  if (!expired) {

    is_stop = (gesture == "STOP"); // gesture can be "STOP", "LIKE", "OK", "YES", "SIX" or empty string

    if(gesture != "") {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[FgsTopicToBlackboard] tick() gesture: '%s' = %s", gesture.c_str(), is_stop ? "BT:SUCCESS" : "BT:FAILURE");
    }
  } else {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,"[FgsTopicToBlackboard] tick()  gesture expired - BT:FAILURE");
  }

  setOutput("is_face_detected", is_face_detected);
  setOutput("is_stop_gesture", is_stop);
  setOutput("face_yaw_error", face_yaw_error);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace slg_bt_plugins

