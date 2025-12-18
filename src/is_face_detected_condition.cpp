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
        // when face is detected, expect steady 2 Hz stream of messages. Not published when face is out of view.
        RCLCPP_WARN(node_->get_logger(), "[IsFaceDetected] sub received face_detected: '%s'", msg->data ? "true" : "false");
        std::lock_guard<std::mutex> lock(mutex_);
        face_detected_ = msg->data;
        last_face_detected_time_ = node_->now();
    });
}

BT::NodeStatus IsFaceDetected::tick()
{
  // The subscription happens, but messages are queued until the node is spun.
  // So, we have to manually process any waiting messages for the BT node:
  rclcpp::spin_some(node_->get_node_base_interface()); // executes any pending callbacks on that node
  
  bool face_detected;
  rclcpp::Time stamp;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    face_detected = face_detected_;
    stamp = last_face_detected_time_;
  }

  const auto now = node_->now();

  // Expire face detection
  if (stamp.nanoseconds() == 0 || (now - stamp) > face_detected_timeout_)
  {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[IsFaceDetected] tick()  face detection expired - BT:FAILURE");

    face_detected = false;
  } else {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[IsFaceDetected] tick()  face_detected: %s", face_detected ? "BT:SUCCESS" : "BT:FAILURE");
  }

  /*
    This node guards the "Turn Toward Face" behavior.
    - Return SUCCESS when: 
        A face is currently detected in the data stream.
        This allows the tree to enter the FaceDetectedSequence.
    - Return FAILURE when:
        No face is detected, or
        if the face tracking stream stops.
    - Why: If the stream ceases and you return SUCCESS,
           the robot will try to run TurnTowardFace without valid data, likely leading to errors or jitter.
           Returning FAILURE ensures the robot falls back to standard NavigateToPose.
  */

  return face_detected
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::FAILURE;
}

}  // namespace slg_bt_plugins

