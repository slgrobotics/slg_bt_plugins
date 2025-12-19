#pragma once

#include "behaviortree_cpp/condition_node.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/illuminance.hpp>

namespace slg_bt_plugins
{

class IsStopGesture : public BT::ConditionNode
{
public:
  IsStopGesture(const std::string & name,
                const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts() { return {}; };

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  // Hack: using Illuminance message for combo info, because we need a time-stamped message in BT plugins.
  rclcpp::Subscription<sensor_msgs::msg::Illuminance>::SharedPtr sub_;


  std::mutex mutex_;
  std::string last_gesture_{""}; // string cannot be atomic, but can be protected by mutex

  rclcpp::Time last_gesture_time_;
  rclcpp::Duration gesture_timeout_{2, 0};  // 2 seconds expiration
};

}  // namespace slg_bt_plugins
