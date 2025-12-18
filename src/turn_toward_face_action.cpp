#include "slg_bt_plugins/turn_toward_face_action.hpp"
#include <cmath>

namespace slg_bt_plugins
{

TurnTowardFace::TurnTowardFace(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error(
      "TurnTowardFace: missing 'node' on blackboard");
  }

  RCLCPP_INFO(node_->get_logger(), "[TurnTowardFace] constructor");

  yaw_error_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/bt/face_yaw_error", 10,
    [this](std_msgs::msg::Float32::SharedPtr msg) {
        RCLCPP_WARN(node_->get_logger(), "[TurnTowardFace] sub received face_yaw_error: '%.3f'", msg->data);
        std::lock_guard<std::mutex> lock(mutex_);
        face_yaw_error_ = msg->data;
        last_yaw_error_time_ = node_->now();
    });

  cmd_vel_pub_ =
    node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::SystemDefaultsQoS());
}

BT::PortsList TurnTowardFace::providedPorts()
{
  // we use ports just for parameters, not live data
  return {
    BT::InputPort<double>(
      "angle_tolerance", 0.1,
      "Acceptable yaw error (radians)"),
    BT::InputPort<double>(
      "max_turn_rate", 0.5,
      "Maximum angular velocity (rad/s)")
  };
}

BT::NodeStatus TurnTowardFace::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "[TurnTowardFace] onStart()");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnTowardFace::onRunning()
{
  std::string angle_key;
  double angle_tolerance;
  double max_turn_rate;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[TurnTowardFace] onRunning()");

  if (!getInput("angle_tolerance", angle_tolerance) ||
      !getInput("max_turn_rate", max_turn_rate)) {

      RCLCPP_INFO(node_->get_logger(), "[TurnTowardFace] onRunning() - missing input parameters");
      return BT::NodeStatus::FAILURE;
  }

  // Prepare command message to be published:
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = "base_link";

  // The subscription happens, but messages are queued until the node is spun.
  // So, we have to manually process any waiting messages for the BT node:
  rclcpp::spin_some(node_->get_node_base_interface());  // executes any pending callbacks on that node
  
  double err_angle;
  rclcpp::Time stamp;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    err_angle = face_yaw_error_; // consume atomic value
    stamp = last_yaw_error_time_;
  }

  const auto now = node_->now();

  // Expire yaw error value
  if (stamp.nanoseconds() == 0 || (now - stamp) > yaw_error_timeout_)
  {
    RCLCPP_INFO(node_->get_logger(), "[TurnTowardFace] yaw error expired");

    // Face alignment stops. Control falls through to navigation immediately
    return BT::NodeStatus::FAILURE;
  }

  // Aligned → stop turning
  if (std::abs(err_angle) < angle_tolerance) {
    cmd_vel_pub_->publish(cmd);
    RCLCPP_INFO(node_->get_logger(), "[TurnTowardFace] face_yaw_error: %.3f  - rotation completed", err_angle);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
       "[TurnTowardFace] - rotating, face_yaw_error: %.3f   tolerance: %.3f", err_angle, angle_tolerance);

  // Turn toward face
  cmd.twist.angular.z = std::copysign(
    std::min(max_turn_rate, std::abs(err_angle)),
    err_angle);

  cmd_vel_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

void TurnTowardFace::onHalted()
{
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = node_->now();
  stop.header.frame_id = "base_link";

  cmd_vel_pub_->publish(stop);

  RCLCPP_INFO(node_->get_logger(), "[TurnTowardFace] onHalted() — stopping rotation");
}

}  // namespace slg_bt_plugins
