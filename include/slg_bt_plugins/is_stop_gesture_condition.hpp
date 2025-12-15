#ifndef SLG_BT_PLUGINS__IS_STOP_GESTURE_CONDITION_HPP_
#define SLG_BT_PLUGINS__IS_STOP_GESTURE_CONDITION_HPP_

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

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void gestureCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string last_gesture_;
};

}  // namespace slg_bt_plugins

#endif  // SLG_BT_PLUGINS__IS_STOP_GESTURE_CONDITION_HPP_