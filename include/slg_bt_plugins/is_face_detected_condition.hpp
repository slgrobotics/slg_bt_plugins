#pragma once

#include "behaviortree_cpp/condition_node.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace slg_bt_plugins
{

class IsFaceDetected : public BT::ConditionNode
{
public:
  IsFaceDetected(const std::string & name,
                const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts() { return {}; };

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

  std::mutex mutex_;
  bool face_detected_{false};

  rclcpp::Time last_face_detected_time_;
  rclcpp::Duration face_detected_timeout_{2, 0};  // 2 seconds expiration
};

}  // namespace slg_bt_plugins
