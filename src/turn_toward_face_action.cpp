#include "slg_bt_plugins/turn_toward_face_action.hpp"
#include <cmath>

namespace slg_bt_plugins
{

TurnTowardFace::TurnTowardFace(const std::string & name,
                const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  // Retrieve Nav2 node from blackboard
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("TurnTowardFace: missing 'node' on blackboard");
  }

  cmd_vel_pub_ =
    node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
}

BT::PortsList TurnTowardFace::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "angle_key", "face_yaw_error",
      "Blackboard key containing yaw error (radians)"),
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
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnTowardFace::onRunning()
{
  std::string angle_key;
  double angle_tolerance;
  double max_turn_rate;

  if (!getInput("angle_key", angle_key) ||
      !getInput("angle_tolerance", angle_tolerance) ||
      !getInput("max_turn_rate", max_turn_rate)) {
    return BT::NodeStatus::FAILURE;
  }

  double angle_error;
  if (!config().blackboard->get(angle_key, angle_error)) {
    return BT::NodeStatus::FAILURE;
  }

  // Aligned → stop
  if (std::abs(angle_error) < angle_tolerance) {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = node_->now();
    stop.header.frame_id = "base_link";
    cmd_vel_pub_->publish(stop);
    return BT::NodeStatus::SUCCESS;
  }

  // Turn toward face
  double turn_rate = std::copysign(
    std::min(max_turn_rate, std::abs(angle_error)),
    angle_error
  );

  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = node_->now();
  twist.header.frame_id = "base_link";
  twist.twist.angular.z = turn_rate;
  cmd_vel_pub_->publish(twist);

  return BT::NodeStatus::RUNNING;
}

void TurnTowardFace::onHalted()
{
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = node_->now();
  stop.header.frame_id = "base_link";
  cmd_vel_pub_->publish(stop);
}

}  // namespace slg_bt_plugins
