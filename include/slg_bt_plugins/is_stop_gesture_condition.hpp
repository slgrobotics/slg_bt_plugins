#pragma once

#include "behaviortree_cpp/condition_node.h"
#include <rclcpp/rclcpp.hpp>

// See CMakeLists.txt for USE_RCLCPP_SUBSCRIPTIONS option
#ifdef USE_RCLCPP_SUBSCRIPTIONS
#include <sensor_msgs/msg/illuminance.hpp>
#endif // USE_RCLCPP_SUBSCRIPTIONS

namespace slg_bt_plugins
{

class IsStopGesture : public BT::ConditionNode
{
public:
  IsStopGesture(const std::string & name,
                const BT::NodeConfiguration & config);

#ifdef USE_RCLCPP_SUBSCRIPTIONS

static BT::PortsList providedPorts() { return {}; };

#else // USE_RCLCPP_SUBSCRIPTIONS

static BT::PortsList providedPorts()
  {
    return BT::PortsList ({
      // Live data from the FgsTopicToBlackboard node
      BT::InputPort<bool>(
      "is_stop_gesture", false,
      "true if a stop gesture is detected, false otherwise")
    });
  }
#endif // USE_RCLCPP_SUBSCRIPTIONS

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

#ifdef USE_RCLCPP_SUBSCRIPTIONS
  // Hack: using Illuminance message for combo info, because we need a time-stamped message in BT plugins.
  rclcpp::Subscription<sensor_msgs::msg::Illuminance>::SharedPtr sub_;


  std::mutex mutex_;
  std::string last_gesture_{""}; // string cannot be atomic, but can be protected by mutex

  rclcpp::Time last_gesture_time_;
  rclcpp::Duration gesture_timeout_{2, 0};  // 2 seconds expiration
#endif // USE_RCLCPP_SUBSCRIPTIONS
};

}  // namespace slg_bt_plugins
