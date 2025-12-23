#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

/**
 * @defgroup BlackboardUtilities Typed Blackboard Utility Nodes
 *
 * Strongly-typed helper nodes for reading and writing BehaviorTree blackboard
 * values without relying on string conversions or implicit type inference.
 */

namespace slg_bt_plugins
{

/**
 * @brief Writes an integer value into the BehaviorTree blackboard.
 *
 * This node stores an integer under a named blackboard key.
 * Useful for counters, retry counts, mode IDs, or numeric state tracking.
 *
 * Behavior:
 *   - Writes the integer value to the blackboard
 *   - Returns SUCCESS on success
 *   - Returns FAILURE if inputs are missing or invalid
 *
 * Usage example:
 * @code
 * <SetBlackboardInt output_key="retry_count" value="3"/>
 * @endcode
 */
class SetBlackboardInt : public BT::SyncActionNode
{
public:
  SetBlackboardInt(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("output_key", "Blackboard key name to set"),
      BT::InputPort<int>("value", "Integer value to write")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Writes a double-precision floating-point value into the BehaviorTree blackboard.
 *
 * This node stores a double under a named blackboard key.
 * Typical uses include thresholds, tolerances, gains, or timing values.
 *
 * Behavior:
 *   - Writes the double value to the blackboard
 *   - Returns SUCCESS on success
 *   - Returns FAILURE if inputs are missing or invalid
 *
 * Usage example:
 * @code
 * <SetBlackboardDouble output_key="yaw_tolerance" value="0.15"/>
 * @endcode
 */
class SetBlackboardDouble : public BT::SyncActionNode
{
public:
  SetBlackboardDouble(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("output_key", "Blackboard key name to set"),
      BT::InputPort<double>("value", "Double value to write")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Writes a string value into the BehaviorTree blackboard.
 *
 * This node stores a string under a named blackboard key.
 * Common uses include gesture names, modes, labels, or state identifiers.
 *
 * Behavior:
 *   - Writes the string value to the blackboard
 *   - Returns SUCCESS on success
 *   - Returns FAILURE if inputs are missing or invalid
 *
 * Usage example:
 * @code
 * <SetBlackboardString output_key="gesture" value="STOP"/>
 * @endcode
 */
class SetBlackboardString : public BT::SyncActionNode
{
public:
  SetBlackboardString(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("output_key", "Blackboard key name to set"),
      BT::InputPort<std::string>("value", "String value to write")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace slg_bt_plugins
