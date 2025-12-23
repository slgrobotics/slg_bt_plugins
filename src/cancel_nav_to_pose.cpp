#include "slg_bt_plugins/cancel_nav_to_pose.hpp"

#include <chrono>
#include <cstring>

namespace slg_bt_plugins
{

CancelNavToPose::CancelNavToPose(const std::string & name,
                                 const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("CancelNavToPose: missing 'node' on blackboard");
  }

  RCLCPP_INFO(node_->get_logger(), "[CancelNavToPose] constructor");
}

BT::NodeStatus CancelNavToPose::onStart()
{
  std::string action_name = "/navigate_to_pose";
  (void)getInput("action_name", action_name);
  if (action_name.empty()) {
    action_name = "/navigate_to_pose";
  }

  const auto service_name = cancel_service_name(action_name);

  // (Re)create client if first time or action_name changed
  if (!client_ || current_service_name_ != service_name) {
    current_service_name_ = service_name;
    client_ = node_->create_client<action_msgs::srv::CancelGoal>(current_service_name_);
    RCLCPP_INFO(node_->get_logger(), "[CancelNavToPose] Using service: %s", current_service_name_.c_str());
  }

  // Don’t hard-block on service availability; keep RUNNING and retry.
  if (!client_->service_is_ready()) {
    (void)client_->wait_for_service(std::chrono::milliseconds(1));
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[CancelNavToPose] Service not ready: %s", current_service_name_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  // Equivalent to: ros2 service call ... "{}"
  // Build request with zero goal_id and zero stamp (cancel all goals).
  auto req = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  std::memset(req->goal_info.goal_id.uuid.data(), 0, req->goal_info.goal_id.uuid.size());
  req->goal_info.stamp.sec = 0;
  req->goal_info.stamp.nanosec = 0;

  future_ = client_->async_send_request(req).future.share();
  request_time_ = node_->now();

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[CancelNavToPose] Sent cancel request to %s", current_service_name_.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CancelNavToPose::onRunning()
{
  int timeout_ms = 500;
  (void)getInput("timeout_ms", timeout_ms);

  if (!future_.valid()) {
    // If something weird happened, re-start
    RCLCPP_WARN(node_->get_logger(), "[CancelNavToPose] Future invalid; restarting");
    return onStart();
  }

  // Non-blocking poll
  if (future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    const double elapsed_ms = (node_->now() - request_time_).seconds() * 1000.0;
    if (elapsed_ms > timeout_ms) {
      RCLCPP_ERROR(node_->get_logger(), "[CancelNavToPose] Timeout waiting for cancel response");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  auto resp = future_.get();
  if (!resp) {
    RCLCPP_ERROR(node_->get_logger(), "[CancelNavToPose] Null response");
    return BT::NodeStatus::FAILURE;
  }

  // return_code == 0 means OK / accepted
  if (resp->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
    RCLCPP_INFO(node_->get_logger(), "[CancelNavToPose] Cancel accepted (%zu goals canceling)",
                resp->goals_canceling.size());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(node_->get_logger(), "[CancelNavToPose] Cancel rejected (return_code=%d)",
               resp->return_code);
  return BT::NodeStatus::FAILURE;
}

void CancelNavToPose::onHalted()
{
  // No special cleanup required.
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[CancelNavToPose] halted");
}

}  // namespace slg_bt_plugins
