#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace slg_bt_plugins
{

/**
 * @brief Condition node that compares a blackboard string value to a literal string.
 *
 * This node retrieves a string from the blackboard using a key name
 * and compares it to a literal string provided in the BT XML.
 *
 * Important:
 *   - The 'key' port is the BLACKBOARD KEY NAME (no braces).
 *   - This differs from most BT nodes that use {var} substitution.
 *
 * Returns:
 *   - SUCCESS if blackboard[key] == compare_to_string
 *   - FAILURE otherwise (including missing key)
 *
 * Typical use cases:
 *   - Gesture recognition (e.g., STOP, OK, YES)
 *   - Mode switching
 *   - State-machine-like branching in BTs
 *
 * Usage example:
 * @code
 * <IsBlackboardStringEqual key="gesture" compare_to_string="STOP"/>
 * @endcode
 */
class IsBlackboardStringEqual : public BT::ConditionNode
{
public:
  IsBlackboardStringEqual(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      // key is the blackboard KEY NAME (no braces): key="gesture"
      BT::InputPort<std::string>("key", "Blackboard key name containing a string (e.g. gesture)"),
      BT::InputPort<std::string>("compare_to_string", "String to compare against (e.g. STOP)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace slg_bt_plugins
