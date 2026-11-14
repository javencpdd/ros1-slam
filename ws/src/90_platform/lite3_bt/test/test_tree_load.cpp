// Verify that BT.CPP 3.8.8 can actually load the shipped XML trees.
// Node implementations are dummies here: this checks XML syntax, tree IDs,
// registration names and port declarations, not runtime behaviour.
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <cstdio>
#include <exception>
#include <string>

class DummyAction : public BT::SyncActionNode
{
public:
  DummyAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("distance", 0.25, "arrival radius"),
            BT::InputPort<double>("timeout", 30.0, "timeout"),
            BT::InputPort<double>("seconds", 1.0, "dwell")};
  }

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

int main(int argc, char** argv)
{
  if (argc < 2) return 2;
  BT::BehaviorTreeFactory factory;
  const char* names[] = {"WaitForGoal", "RequestPath", "WaitForPath",
                         "FollowPath", "SendGoal", "WaitArrival",
                         "Dwell", "Stop", "Recovery", "CancelStop",
                         "IsCancelled", "IsNotCancelled", "CheckLocalization"};
  for (const char* name : names) factory.registerNodeType<DummyAction>(name);
  try
  {
    BT::Tree tree = factory.createTreeFromFile(argv[1]);
    if (tree.nodes.size() < 5)
    {
      std::printf("TREE FAIL %s: only %zu nodes\n", argv[1], tree.nodes.size());
      return 1;
    }
    std::printf("TREE OK %s (%zu nodes)\n", argv[1], tree.nodes.size());
    return 0;
  }
  catch (const std::exception& error)
  {
    std::printf("TREE FAIL %s: %s\n", argv[1], error.what());
    return 1;
  }
}
