#pragma once

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

namespace slg_bt_plugins
{

class TurnTowardFace : public BT::StatefulActionNode
{
public:
  TurnTowardFace(const std::string & name,
                 const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_error_sub_;
  double face_yaw_error_{0.0};
};

}  // namespace slg_bt_plugins
