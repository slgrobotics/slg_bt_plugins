#ifndef SLG_BT_PLUGINS__TURN_TOWARD_FACE_ACTION_HPP_
#define SLG_BT_PLUGINS__TURN_TOWARD_FACE_ACTION_HPP_

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace slg_bt_plugins
{

class TurnTowardFace : public BT::StatefulActionNode
{
public:
  TurnTowardFace(const std::string & name,
                 const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  //BT::NodeStatus tick() override;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
};

}  // namespace slg_bt_plugins

#endif  // SLG_BT_PLUGINS__TURN_TOWARD_FACE_ACTION_HPP_