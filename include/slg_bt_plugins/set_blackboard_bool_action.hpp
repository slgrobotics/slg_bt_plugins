#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

/**
 * @brief Writes a boolean value into the BehaviorTree blackboard.
 *
 * This node sets a named blackboard key to a boolean value.
 * It is typically used to implement latches, flags, or state variables
 * that persist across BT ticks.
 *
 * Behavior:
 *   - Writes the value to the blackboard under output_key
 *   - Returns SUCCESS on successful write
 *   - Returns FAILURE if required inputs are missing
 *
 * Usage example:
 * @code
 * <SetBlackboardBool output_key="face_pause_active" value="true"/>
 * @endcode
 */
namespace slg_bt_plugins
{

class SetBlackboardBool : public BT::SyncActionNode
{
public:
  SetBlackboardBool(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      // Use a literal key name (not {var}) here:
      // <SetBlackboardBool output_key="face_pause_active" value="true"/>
      BT::InputPort<std::string>("output_key", "Blackboard key name to set"),
      BT::InputPort<bool>("value", "Boolean value to write")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace slg_bt_plugins
