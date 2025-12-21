#pragma once

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>

namespace slg_bt_plugins
{

class DumpBlackboardData : public BT::SyncActionNode
{
public:
  DumpBlackboardData(const std::string & name,
                const BT::NodeConfiguration & config);

static BT::PortsList providedPorts()
  {
    return BT::PortsList ({
      // Live data from the FgsTopicToBlackboard node
      BT::InputPort<bool>(
        "is_face_detected",
        "true if a face is detected, false otherwise"),

      BT::InputPort<float>(
        "face_yaw_error",
        "an angle in radians to the face when detected, zero otherwise"),

      BT::InputPort<std::string>(
        "gesture",
        "actual gesture as detected by the perception adapter"),

      BT::InputPort<bool>(
        "is_stop_gesture",
        "true if a STOP gesture is detected, false otherwise"),

      BT::InputPort<bool>(
        "is_like_gesture",
        "true if a LIKE gesture is detected, false otherwise"),

      BT::InputPort<bool>(
        "is_ok_gesture",
        "true if an OK gesture is detected, false otherwise"),

      BT::InputPort<bool>(
        "is_yes_gesture",
        "true if a YES gesture is detected, false otherwise"),

      BT::InputPort<bool>(
        "is_six_gesture",
        "true if a SIX gesture is detected, false otherwise")
    });
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

};

}  // namespace slg_bt_plugins
