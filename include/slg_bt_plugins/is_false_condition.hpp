#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Condition node that returns SUCCESS when a boolean input is false.
 *
 * This node reads a boolean value from an input port and evaluates it.
 * It returns:
 *   - SUCCESS if the value is false
 *   - FAILURE if the value is true or missing
 *
 * Typical use cases:
 *   - Clearing latches
 *   - Inverting boolean conditions without an Inverter decorator
 *   - Expressive guard conditions in Fallback / Sequence nodes
 *
 * Usage example:
 * @code
 * <IsFalse key="{is_face_detected}"/>
 * @endcode
 */
namespace slg_bt_plugins
{

class IsFalse : public BT::ConditionNode
{
public:
  IsFalse(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      // In XML you pass: key="{some_bool}"
      BT::InputPort<bool>("key", "Boolean value to test (SUCCESS if false)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace slg_bt_plugins
