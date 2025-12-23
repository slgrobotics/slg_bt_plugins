#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>

namespace slg_bt_plugins
{

class IsTrue : public BT::ConditionNode
{
public:
  IsTrue(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      // In XML you pass: key="{some_bool}"
      BT::InputPort<bool>("key", "Boolean value to test (SUCCESS if true)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace slg_bt_plugins
