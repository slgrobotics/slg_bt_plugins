#pragma once

#include "behaviortree_cpp/condition_node.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string last_gesture_{""};
};

}  // namespace slg_bt_plugins
