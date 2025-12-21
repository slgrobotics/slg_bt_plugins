#pragma once

//#define USE_RCLCPP_SUBSCRIPTIONS

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>

#ifdef USE_RCLCPP_SUBSCRIPTIONS
#include <sensor_msgs/msg/illuminance.hpp>
#endif // USE_RCLCPP_SUBSCRIPTIONS

#include <geometry_msgs/msg/twist_stamped.hpp>

namespace slg_bt_plugins
{

class TurnTowardFace : public BT::StatefulActionNode
{
public:
  TurnTowardFace(const std::string & name,
                 const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList ({
#ifndef USE_RCLCPP_SUBSCRIPTIONS
        // Live data from the FgsTopicToBlackboard node
        BT::InputPort<float>(
        "face_yaw_error", 0.0f,
        "an angle in radians to the face when detected, zero otherwise"),
#endif // USE_RCLCPP_SUBSCRIPTIONS
        // we use ports below just for parameters, not live data
        BT::InputPort<double>(
        "angle_tolerance", 0.1,
        "Acceptable yaw error (radians)"),
        BT::InputPort<double>(
        "max_turn_rate", 0.5,
        "Maximum angular velocity (rad/s)")
    });
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

#ifdef USE_RCLCPP_SUBSCRIPTIONS
  // Hack: using Illuminance message for combo info, because we need a time-stamped message in BT plugins.
  rclcpp::Subscription<sensor_msgs::msg::Illuminance>::SharedPtr yaw_error_sub_;


  std::mutex mutex_;
  float face_yaw_error_{0.0f};

  rclcpp::Time last_yaw_error_time_;
  rclcpp::Duration yaw_error_timeout_{2, 0};  // 2 seconds expiration
#endif // USE_RCLCPP_SUBSCRIPTIONS

}; 

} // namespace slg_bt_plugins
