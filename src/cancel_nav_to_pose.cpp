#include "slg_bt_plugins/cancel_nav_to_pose.hpp"

#include <chrono>

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
    client_ = node_->create_client<action_msgs::srv::CancelGoal>(current_service_name_, true);
    RCLCPP_INFO(node_->get_logger(), "[CancelNavToPose] Using service: %s", current_service_name_.c_str());
  }

  future_ = {};
  request_time_ = std::chrono::steady_clock::now();
  return onRunning();
}

BT::NodeStatus CancelNavToPose::onRunning()
{
  int timeout_ms = 500;
  (void)getInput("timeout_ms", timeout_ms);
  if (std::chrono::steady_clock::now() - request_time_ >
    std::chrono::milliseconds(timeout_ms))
  {
    future_ = {};
    client_.reset();
    RCLCPP_ERROR(node_->get_logger(), "[CancelNavToPose] Cancellation timed out");
    return BT::NodeStatus::FAILURE;
  }

  if (!future_.valid()) {
    if (!client_->wait_for_service(std::chrono::milliseconds(1))) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[CancelNavToPose] Service not ready: %s", current_service_name_.c_str());
      return BT::NodeStatus::RUNNING;
    }
    auto req = std::make_shared<action_msgs::srv::CancelGoal::Request>();
    // Zero UUID and timestamp request cancellation of all goals on this server.
    future_ = client_->async_call(req);
  }

  // Process only this client's private callback group, including in no-pump trees.
  (void)client_->spin_until_complete(future_, std::chrono::milliseconds(0));
  // Non-blocking poll
  if (future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
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
  future_ = {};
  client_.reset();
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[CancelNavToPose] halted");
}

}  // namespace slg_bt_plugins
