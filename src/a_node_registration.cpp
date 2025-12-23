
#include <behaviortree_cpp/bt_factory.h>

#include "slg_bt_plugins/fgs_to_bt_node.hpp"
#include "slg_bt_plugins/dump_blackboard_data.hpp"
#include "slg_bt_plugins/is_face_detected_condition.hpp"
#include "slg_bt_plugins/is_stop_gesture_condition.hpp"
#include "slg_bt_plugins/turn_toward_face_action.hpp"
#include "slg_bt_plugins/cancel_nav_to_pose.hpp"
#include "slg_bt_plugins/is_true_condition.hpp"
#include "slg_bt_plugins/set_blackboard_bool_action.hpp"
#include "slg_bt_plugins/is_false_condition.hpp"
#include "slg_bt_plugins/set_blackboard_typed_actions.hpp"
#include "slg_bt_plugins/is_blackboard_string_equal_condition.hpp"

// Register all custom node types to BehaviorTreeFactory in one place, outside the namespace

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<slg_bt_plugins::FgsTopicToBlackboard>("FgsTopicToBlackboard");
  factory.registerNodeType<slg_bt_plugins::DumpBlackboardData>("DumpBlackboardData");
  factory.registerNodeType<slg_bt_plugins::TurnTowardFace>("TurnTowardFace");
  factory.registerNodeType<slg_bt_plugins::IsStopGesture>("IsStopGesture");
  factory.registerNodeType<slg_bt_plugins::IsFaceDetected>("IsFaceDetected");
  factory.registerNodeType<slg_bt_plugins::CancelNavToPose>("CancelNavToPose");
  factory.registerNodeType<slg_bt_plugins::IsTrue>("IsTrue");
  factory.registerNodeType<slg_bt_plugins::SetBlackboardBool>("SetBlackboardBool");
  factory.registerNodeType<slg_bt_plugins::IsFalse>("IsFalse");
  factory.registerNodeType<slg_bt_plugins::SetBlackboardInt>("SetBlackboardInt");
  factory.registerNodeType<slg_bt_plugins::SetBlackboardDouble>("SetBlackboardDouble");
  factory.registerNodeType<slg_bt_plugins::SetBlackboardString>("SetBlackboardString");
  factory.registerNodeType<slg_bt_plugins::IsBlackboardStringEqual>("IsBlackboardStringEqual");
}
