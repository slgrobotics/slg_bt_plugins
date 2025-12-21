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

#ifdef USE_RCLCPP_SUBSCRIPTIONS
  yaw_error_sub_ = node_->create_subscription<sensor_msgs::msg::Illuminance>(
    "/bt/face_gesture_detect",
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
    // Note: the Lambda callback will be called when BT node is spun and only then the messages will be received, including the queued stale ones
    [this](sensor_msgs::msg::Illuminance::SharedPtr msg) {
        // while face is detected, expect steady 2 Hz stream of messages.
        RCLCPP_WARN(node_->get_logger(), "[TurnTowardFace] sub received face_yaw_error: '%.3f'", msg->variance);
        std::lock_guard<std::mutex> lock(mutex_);
        face_yaw_error_ = msg->variance; // using variance field for yaw error
        last_yaw_error_time_ = rclcpp::Time(msg->header.stamp, node_->get_clock()->get_clock_type()); // time when the message was sent
    });
#endif // USE_RCLCPP_SUBSCRIPTIONS

  cmd_vel_pub_ =
    node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::SystemDefaultsQoS());
}

BT::NodeStatus TurnTowardFace::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "[TurnTowardFace] onStart()");
  return BT::NodeStatus::RUNNING;
}

/***
 * 
 * Since this is an asynchronous Action Node paired with a condition in a sequence,
 *  its status controls how long the robot stays "locked" in the turning behavior.
 * 
 * Return RUNNING while:
 *     The robot is actively rotating toward the person but has not yet reached the angle_tolerance.
 * 
 * Return SUCCESS when:
 *     The robot has successfully aligned within the angle_tolerance.
 * 
 *   Effect:
 *     Once this returns SUCCESS, the FaceDetectedSequence completes. 
 *     On the next tick, the tree restarts at the top. 
 *     If IsFaceDetected is still SUCCESS, it will start turning again (maintaining alignment).
 * 
 * Return FAILURE when:
 *     The target yaw data is missing for too long,
 *     the robot cannot physically turn, or
 *     the face is lost during the turn (yaw data is missing).
 * 
 *   Effect:
 *     This immediately breaks the sequence, allowing the ReactiveFallback to drop down to NavigateToPose.
 * 
 ***/
BT::NodeStatus TurnTowardFace::onRunning()
{
  std::string angle_key;
  double angle_tolerance;
  double max_turn_rate;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[TurnTowardFace] onRunning()");

  if (!getInput("angle_tolerance", angle_tolerance) ||
      !getInput("max_turn_rate", max_turn_rate)) {

      RCLCPP_WARN(node_->get_logger(), "[TurnTowardFace] onRunning() - missing input parameters");
      return BT::NodeStatus::FAILURE;
  }

  // Prepare command message to be published:
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = "base_link";

  double err_angle;

#ifdef USE_RCLCPP_SUBSCRIPTIONS
  // The subscription happens, but messages are queued until the node is spun.
  // So, we have to manually process any waiting messages for the BT node:
  rclcpp::spin_some(node_->get_node_base_interface());  // executes any pending callbacks on that node
  
  rclcpp::Time msg_timestamp;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    err_angle = face_yaw_error_; // consume atomic value
    msg_timestamp = last_yaw_error_time_;
  }

  const auto now = node_->now();

  // Expire yaw error value
  if (msg_timestamp.nanoseconds() == 0 || (now - msg_timestamp) > yaw_error_timeout_)
  {
    RCLCPP_INFO(node_->get_logger(), "[TurnTowardFace] yaw error expired - BT:FAILURE");

    // Face alignment stops. Control falls through to navigation immediately
    return BT::NodeStatus::FAILURE;
  }
#else // USE_RCLCPP_SUBSCRIPTIONS

  if (!getInput("face_yaw_error", err_angle)) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[TurnTowardFace] onRunning() - missing face_yaw_error");
      return BT::NodeStatus::RUNNING; // stay in sequence. Is data pump working?
  }

#endif // USE_RCLCPP_SUBSCRIPTIONS

  // Aligned → stop turning
  if (std::abs(err_angle) < angle_tolerance) {
    cmd_vel_pub_->publish(cmd);
    RCLCPP_INFO(node_->get_logger(), "[TurnTowardFace] face_yaw_error: %.3f < %.3f - rotation completed - BT:SUCCESS", err_angle, angle_tolerance);

    return BT::NodeStatus::SUCCESS;
  }

  double turn_rate_factor = 1.0;

  double turn_rate = err_angle * turn_rate_factor; // angle -> radians/sec
  
  turn_rate = std::copysign(std::min(max_turn_rate, std::abs(turn_rate)), err_angle);  // limit to max_turn_rate with sign

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
       "[TurnTowardFace] - rotating, face_yaw_error: %.3f   tolerance: %.3f   turn_rate: %.3f - BT:RUNNING", err_angle, angle_tolerance, turn_rate);

  // Turn toward face
  cmd.twist.angular.z = turn_rate;

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
