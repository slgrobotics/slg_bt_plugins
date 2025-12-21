#include "slg_bt_plugins/dump_blackboard_data.hpp"

namespace slg_bt_plugins
{

DumpBlackboardData::DumpBlackboardData(const std::string & name,
                const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error(
      "DumpBlackboardData: missing 'node' on blackboard");
  }

  RCLCPP_INFO(node_->get_logger(), "[DumpBlackboardData] constructor");

}

BT::NodeStatus DumpBlackboardData::tick()
{
  bool face_detected;
  float err_angle;
  std::string gesture;
  bool is_stop;
  bool is_like;
  bool is_ok;
  bool is_yes;
  bool is_six;

  int trace_throttle_period_ms = 2000;

  if (!getInput("is_face_detected", face_detected)) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick() - missing 'is_face_detected' input on blackboard");
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick()  face_detected: %s", face_detected ? "true" : "false");

  if (!getInput("face_yaw_error", err_angle)) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick() - missing 'face_yaw_error' input on blackboard");
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick()  face_yaw_error: %.3f", (double)err_angle);

  if (!getInput("gesture", gesture)) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick() - missing 'gesture' input on blackboard");
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick()  gesture: %s", gesture.c_str());

  if (!getInput("is_stop_gesture", is_stop)) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick() - missing 'is_stop_gesture' input on blackboard");
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick()  is_stop_gesture: %s", is_stop ? "true" : "false");

  if (!getInput("is_like_gesture", is_like)) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick() - missing 'is_like_gesture' input on blackboard");
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick()  is_like_gesture: %s", is_like ? "true" : "false");

  if (!getInput("is_ok_gesture", is_ok)) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick() - missing 'is_ok_gesture' input on blackboard");
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick()  is_ok_gesture: %s", is_ok ? "true" : "false");

  if (!getInput("is_yes_gesture", is_yes)) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick() - missing 'is_yes_gesture' input on blackboard");
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick()  is_yes_gesture: %s", is_yes ? "true" : "false");

  if (!getInput("is_six_gesture", is_six)) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick() - missing 'is_six_gesture' input on blackboard");
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), trace_throttle_period_ms, "[DumpBlackboardData] tick()  is_six_gesture: %s", is_six ? "true" : "false");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace slg_bt_plugins