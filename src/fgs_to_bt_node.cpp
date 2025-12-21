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

  // Only the latest sample; No replay; No backlog
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort().durability_volatile();

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
  
  bool valid = false;
  bool is_face_detected = false;
  float face_yaw_error = 0.0;
  std::string gesture;

  bool is_stop = false;
  bool is_like = false;
  bool is_ok = false;
  bool is_yes = false;
  bool is_six = false;

  rclcpp::Duration message_timeout_{1, 0};  // 1 second expiration for messages

  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Valid means recent. Messages come at ~10 Hz; we assume 1000 ms is recent enough.
    valid = last_message_time_.nanoseconds() != 0 &&
                (node_->now() - last_message_time_) <= message_timeout_;

    if (valid) {
        is_face_detected = is_face_detected_;
        face_yaw_error = face_yaw_error_;
        gesture = last_gesture_;
    } else {
        is_face_detected = false;
        face_yaw_error = 0.0f;
        gesture.clear();
    }
  }

  if (valid) {
    is_stop = (gesture == "STOP"); // gesture can be "STOP", "LIKE", "OK", "YES", "SIX" or empty string
    is_like = (gesture == "LIKE");
    is_ok = (gesture == "OK");
    is_yes = (gesture == "YES");
    is_six = (gesture == "SIX");
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,"[FgsTopicToBlackboard] tick()  face & gesture message stale or missing");
  }

  //if(is_face_detected) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[FgsTopicToBlackboard] tick() is_face_detected: '%s'   face_yaw_error: %.5f   gesture: '%s'",
       (is_face_detected ? "true" : "false"), face_yaw_error, gesture.c_str());
  //}

  // Note: the output values are set even if the message is stale or missing.
  // The tree logic will decide what to do based on these values.

  setOutput("is_face_detected", is_face_detected);
  setOutput("face_yaw_error", face_yaw_error);

  setOutput("gesture", gesture);

  setOutput("is_stop_gesture", is_stop);
  setOutput("is_like_gesture", is_like);
  setOutput("is_ok_gesture", is_ok);
  setOutput("is_yes_gesture", is_yes);
  setOutput("is_six_gesture", is_six);

  // This node is not a decision; it's just a data pump
  // It should never block the tree
  return BT::NodeStatus::SUCCESS;
}

}  // namespace slg_bt_plugins

