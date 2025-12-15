#ifndef SLG_BT_PLUGINS__IS_FACE_DETECTED_CONDITION_HPP_
#define SLG_BT_PLUGINS__IS_FACE_DETECTED_CONDITION_HPP_

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

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void faceDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  bool face_detected_;
};

}  // namespace slg_bt_plugins

#endif  // SLG_BT_PLUGINS__IS_FACE_DETECTED_CONDITION_HPP_