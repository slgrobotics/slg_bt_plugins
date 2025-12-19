#pragma once

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/illuminance.hpp>

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

  // Hack: using Illuminance message for combo info, because we need a time-stamped message in BT plugins.
  rclcpp::Subscription<sensor_msgs::msg::Illuminance>::SharedPtr yaw_error_sub_;


  std::mutex mutex_;
  double face_yaw_error_{0.0};

  rclcpp::Time last_yaw_error_time_;
  rclcpp::Duration yaw_error_timeout_{2, 0};  // 2 seconds expiration

}; 

} // namespace slg_bt_plugins
