#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

namespace slg_bt_plugins
{

/** @brief A BehaviorTree action node that cancels a running Nav2 action.
 *
 * This node will attempt to cancel the specified Nav2 action by calling its
 * `_action/cancel_goal` service. The action name and timeout for waiting for the
 * service response can be specified as input ports.
 * 
 * It does the same as the following CLI command:
 *     ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
 */
class CancelNavToPose : public BT::StatefulActionNode
{
public:
  CancelNavToPose(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("action_name", "/navigate_to_pose",
        "Nav2 action name to cancel (e.g. /navigate_to_pose)"),
      BT::InputPort<int>("timeout_ms", 500,
        "Timeout waiting for cancel_goal service response (ms)")
    };
  }

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  std::string cancel_service_name(const std::string & action_name) const
  {
    return action_name + "/_action/cancel_goal";
  }

  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr client_;
  std::string current_service_name_;

  std::shared_future<action_msgs::srv::CancelGoal::Response::SharedPtr> future_;
  rclcpp::Time request_time_;
};

}  // namespace slg_bt_plugins
