
#include <behaviortree_cpp/bt_factory.h>

#include "slg_bt_plugins/fgs_to_bt_node.hpp"
#include "slg_bt_plugins/is_face_detected_condition.hpp"
#include "slg_bt_plugins/is_stop_gesture_condition.hpp"
#include "slg_bt_plugins/turn_toward_face_action.hpp"

// Register all custom node types to BehaviorTreeFactory in one place, outside the namespace

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<slg_bt_plugins::FgsTopicToBlackboard>("FgsTopicToBlackboard");
  factory.registerNodeType<slg_bt_plugins::TurnTowardFace>("TurnTowardFace");
  factory.registerNodeType<slg_bt_plugins::IsStopGesture>("IsStopGesture");
  factory.registerNodeType<slg_bt_plugins::IsFaceDetected>("IsFaceDetected");
}
