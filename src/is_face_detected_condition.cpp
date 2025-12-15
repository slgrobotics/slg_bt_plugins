#include "slg_bt_plugins/is_face_detected_condition.hpp"
#include <behaviortree_cpp/bt_factory.h>

namespace slg_bt_plugins
{

IsFaceDetected::IsFaceDetected(const std::string & name,
                const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config), face_detected_(false)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/face_detected", 10,
    std::bind(&IsFaceDetected::faceDetectedCallback, this, std::placeholders::_1));
}

BT::PortsList IsFaceDetected::providedPorts()
{
  return {};
}

BT::NodeStatus IsFaceDetected::tick()
{
  if (face_detected_) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

void IsFaceDetected::faceDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  face_detected_ = msg->data;
}

}  // namespace slg_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<slg_bt_plugins::IsFaceDetected>("IsFaceDetected");
}
