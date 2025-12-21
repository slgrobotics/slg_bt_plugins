#pragma once

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/illuminance.hpp>

namespace slg_bt_plugins
{

/** @brief This node subscribes to the FGS perception adapter topic and writes the
 *        detected gesture and related flags and face yaw to the blackboard.
 */
class FgsTopicToBlackboard : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for slg_bt_plugins::FgsTopicToBlackboard
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param config  BT node configuration
   */
  FgsTopicToBlackboard(const std::string & xml_tag_name,
                const BT::NodeConfiguration & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic_name",
        "perception adapter topic name to subscribe to, usually '/bt/face_gesture_detect'"),

        BT::OutputPort<bool>(
        "is_face_detected",
        "true if a face is detected, false otherwise"),

        BT::OutputPort<float>(
        "face_yaw_error",
        "an angle in radians to the face when detected, zero otherwise"),

        BT::OutputPort<std::string>(
        "gesture",
        "actual gesture as detected by the perception adapter"),

        BT::OutputPort<bool>(
        "is_stop_gesture",
        "true if a STOP gesture is detected, false otherwise"),

        BT::OutputPort<bool>(
        "is_like_gesture",
        "true if a LIKE gesture is detected, false otherwise"),

        BT::OutputPort<bool>(
        "is_ok_gesture",
        "true if an OK gesture is detected, false otherwise"),

        BT::OutputPort<bool>(
        "is_yes_gesture",
        "true if a YES gesture is detected, false otherwise"),

        BT::OutputPort<bool>(
        "is_six_gesture",
        "true if a SIX gesture is detected, false otherwise")
    };
  }

private:

  BT::NodeStatus tick() override;

  void callbackFgsPerceptionAdapterMessage(const sensor_msgs::msg::Illuminance::SharedPtr msg);

  // Hack: using Illuminance message for combo info, because we need a time-stamped message in BT plugins.
  rclcpp::Subscription<sensor_msgs::msg::Illuminance>::SharedPtr sub_;

  rclcpp::Node::SharedPtr node_;

  std::string topic_name_;

  // thread safe variables:
  std::mutex mutex_;
  rclcpp::Time last_message_time_;
  std::string last_gesture_ {""};
  bool is_face_detected_{false};
  double face_yaw_error_{0.0};
};

}  // namespace slg_bt_plugins
