#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/utils/shared_library.h>
#include <nav2_behavior_tree/plugins_list.hpp>
#include <filesystem>
#include <iostream>
#include <map>
#include <stdexcept>

struct Scenario
{
  bool stop = false;
  bool face = false;
  BT::NodeStatus navigation = BT::NodeStatus::RUNNING;
  BT::NodeStatus cancel = BT::NodeStatus::RUNNING;
  BT::NodeStatus turn = BT::NodeStatus::RUNNING;
  std::map<std::string, int> ticks;
  std::map<std::string, int> halts;
};

// Keep the installed plugin manifests (including their actual ports), but
// replace ROS leaves to exercise XML control flow without live action servers.
class TestLeaf : public BT::ActionNodeBase
{
public:
  TestLeaf(const std::string & name, const BT::NodeConfig & config,
    const std::string & id, Scenario & scenario)
  : BT::ActionNodeBase(name, config), id_(id), scenario_(scenario) {}

  BT::NodeStatus tick() override
  {
    ++scenario_.ticks[id_];
    if (id_ == "IsStopGesture") {
      return scenario_.stop ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    if (id_ == "IsFaceDetected") {
      return scenario_.face ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    if (id_ == "TurnTowardFace") return scenario_.turn;
    if (id_ == "CancelNavToPose") {
      const auto result = scenario_.cancel;
      // A real StatefulActionNode starts a new asynchronous attempt on retry.
      if (result == BT::NodeStatus::FAILURE) scenario_.cancel = BT::NodeStatus::RUNNING;
      return result;
    }
    if (id_ == "FollowPath") return scenario_.navigation;
    if (id_ == "GlobalUpdatedGoal" || id_ == "IsGoalNearby" ||
      id_.find("WouldA") == 0) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
    ++scenario_.halts[id_];
    resetStatus();
  }

private:
  std::string id_;
  Scenario & scenario_;
};

void require(bool value, const std::string & message)
{
  if (!value) throw std::runtime_error(message);
}

int main(int argc, char ** argv)
{
  if (argc != 4) return 2;
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(argv[1]);
  BT::SharedLibrary loader;
  for (const auto & name : BT::splitString(nav2::details::BT_BUILTIN_PLUGINS, ';')) {
    factory.registerFromPlugin(
      (std::filesystem::path(argv[3]) / loader.getOSName(std::string(name))).string());
  }
  Scenario scenario;
  const auto manifests = factory.manifests();
  for (const auto & entry : manifests) {
    if (factory.builtinNodes().count(entry.first)) continue;
    if (entry.second.type != BT::NodeType::ACTION &&
      entry.second.type != BT::NodeType::CONDITION) continue;
    factory.unregisterBuilder(entry.first);
    factory.registerBuilder(entry.second,
      [&scenario, id = entry.first](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<TestLeaf>(name, config, id, scenario);
      });
  }
  const std::filesystem::path directory(argv[2]);
#ifdef USE_RCLCPP_SUBSCRIPTIONS
  const std::vector<std::string> files = {
    "fgs_data_to_blackboard.xml", "dump_blackboard_data.xml",
    "no_pump/turn_to_human.xml", "no_pump/face_aware_nav.xml"};
#else
  const std::vector<std::string> files = {
    "fgs_data_to_blackboard.xml", "dump_blackboard_data.xml", "minimal_pump.xml",
    "nav_to_stop.xml", "face_aware_nav.xml"};
#endif
  for (const auto & file : files) {
    factory.clearRegisteredBehaviorTrees();
    scenario = Scenario{};
    auto tree = factory.createTreeFromFile((directory / file).string());
    require(tree.tickOnce() == BT::NodeStatus::RUNNING, file + ": unexpectedly completed");
    require(tree.tickOnce() == BT::NodeStatus::RUNNING, file + ": stopped monitoring");
    const bool navigation = file.find("face_aware_nav") != std::string::npos;
    const bool turning = navigation || file.find("turn_to_human") != std::string::npos || file == "nav_to_stop.xml";
    if (turning) {
      scenario.face = true;
      require(tree.tickOnce() == BT::NodeStatus::RUNNING, file + ": face completed goal");
      require(scenario.ticks["TurnTowardFace"] > 0, file + ": did not turn");
      if (navigation) require(scenario.halts["FollowPath"] > 0, file + ": controller not halted");
      scenario.turn = BT::NodeStatus::SUCCESS;
      require(tree.tickOnce() == BT::NodeStatus::RUNNING, file + ": alignment completed goal");
      scenario.turn = BT::NodeStatus::RUNNING;
      tree.tickOnce();
      scenario.face = false;
      const int turns_before_face_loss = scenario.ticks["TurnTowardFace"];
      const int halts_before_face_loss = scenario.halts["TurnTowardFace"];
      require(tree.tickOnce() == BT::NodeStatus::RUNNING, file + ": face loss completed goal");
      require(scenario.ticks["TurnTowardFace"] == turns_before_face_loss,
        file + ": face condition was not rechecked");
      require(scenario.halts["TurnTowardFace"] > halts_before_face_loss,
        file + ": face loss did not halt turning");
      scenario.face = true;
      tree.tickOnce();
      scenario.stop = true;
      const int turn_ticks = scenario.ticks["TurnTowardFace"];
      require(tree.tickOnce() == BT::NodeStatus::RUNNING, file + ": STOP completed goal");
      require(scenario.ticks["TurnTowardFace"] == turn_ticks, file + ": turned during STOP");
      require(scenario.halts["TurnTowardFace"] > 0, file + ": turn not halted");
      scenario.face = false;
      if (navigation) {
        const int follow_ticks = scenario.ticks["FollowPath"];
        tree.tickOnce();
        require(scenario.ticks["FollowPath"] == follow_ticks, file + ": navigated during STOP");
        scenario.stop = false;
        require(tree.tickOnce() == BT::NodeStatus::RUNNING, file + ": cancellation stopped when STOP cleared");
        require(scenario.ticks["FollowPath"] == follow_ticks, file + ": resumed canceled goal");
        scenario.cancel = BT::NodeStatus::FAILURE;
        tree.tickOnce();
        require(scenario.ticks["FollowPath"] == follow_ticks, file + ": resumed after cancellation failure");
        scenario.cancel = BT::NodeStatus::SUCCESS;
        require(tree.tickOnce() == BT::NodeStatus::RUNNING, file + ": cancellation reported goal success");
        const int cancel_ticks = scenario.ticks["CancelNavToPose"];
        tree.tickOnce();
        require(scenario.ticks["CancelNavToPose"] == cancel_ticks, file + ": repeated accepted cancellation");
        require(scenario.ticks["FollowPath"] == follow_ticks, file + ": resumed after accepted cancellation");
        // Nav2 halts this tree on cancellation; a subsequent goal starts afresh.
        tree.haltTree();
        scenario = Scenario{};
        require(tree.tickOnce() == BT::NodeStatus::RUNNING, file + ": new goal did not start");
        scenario.navigation = BT::NodeStatus::SUCCESS;
        require(tree.tickOnce() == BT::NodeStatus::SUCCESS, file + ": navigation completion masked");
        tree.haltTree();
        scenario.navigation = BT::NodeStatus::FAILURE;
        require(tree.tickOnce() == BT::NodeStatus::FAILURE, file + ": navigation failure masked");
      } else {
        scenario.stop = false;
        tree.tickOnce();
        require(scenario.ticks["TurnTowardFace"] == turn_ticks, file + ": turned without a face");
      }
    }
#ifndef USE_RCLCPP_SUBSCRIPTIONS
    require(scenario.ticks["FgsTopicToBlackboard"] >= 2,
      file + ": perception pump was starved");
#endif
    tree.haltTree();
    std::cout << "Passed " << file << '\n';
  }
}
