#pragma once

#include "behaviortree_cpp/condition_node.h"
#include <rclcpp/rclcpp.hpp>

// See CMakeLists.txt for USE_RCLCPP_SUBSCRIPTIONS option
#ifdef USE_RCLCPP_SUBSCRIPTIONS
#include <sensor_msgs/msg/illuminance.hpp>
#endif // USE_RCLCPP_SUBSCRIPTIONS

namespace slg_bt_plugins
{

/**
 * @brief Condition node that checks if a face is detected.
 *
 * This node can be used to check if a human's face is detected by the robot's sensors.
 * It uses the `is_face_detected` input port to determine if a face is detected.
 * If the face is detected, it returns `SUCCESS`; otherwise, it returns `FAILURE`.
 */
class IsFaceDetected : public BT::ConditionNode
{
public:
  IsFaceDetected(const std::string & name,
                const BT::NodeConfiguration & config);

#ifdef USE_RCLCPP_SUBSCRIPTIONS

static BT::PortsList providedPorts() { return {}; };

#else // USE_RCLCPP_SUBSCRIPTIONS

static BT::PortsList providedPorts()
  {
    return BT::PortsList ({
      // Live data from the FgsTopicToBlackboard node
      BT::InputPort<bool>(
        "is_face_detected", false,
        "true if a face is detected, false otherwise")
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
  bool face_detected_{false};

  rclcpp::Time last_face_detected_time_;
  rclcpp::Duration face_detected_timeout_{2, 0};  // 2 seconds expiration

#endif // USE_RCLCPP_SUBSCRIPTIONS
};

}  // namespace slg_bt_plugins
