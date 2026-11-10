#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

namespace
{
struct SharedState
{
  std::mutex mutex;
  geometry_msgs::PoseStamped goal;
  geometry_msgs::PoseStamped pose;
  bool has_goal = false;
  bool has_pose = false;
  uint64_t goal_generation = 0;
  uint64_t goal_consumed = 0;
  std::string last_event = "idle";
  ros::Publisher goal_pub;
  ros::Publisher stop_pub;
};

SharedState g_state;
std::chrono::steady_clock::time_point g_start;
double g_timeout_s = 60.0;

class WaitForGoal : public BT::StatefulActionNode
{
public:
  WaitForGoal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override { return takeGoal(); }
  BT::NodeStatus onRunning() override { return takeGoal(); }
  void onHalted() override {}

private:
  BT::NodeStatus takeGoal()
  {
    std::lock_guard<std::mutex> lock(g_state.mutex);
    if (g_state.goal_generation <= g_state.goal_consumed)
      return BT::NodeStatus::RUNNING;
    g_state.goal_consumed = g_state.goal_generation;
    g_state.last_event = "goal";
    return BT::NodeStatus::SUCCESS;
  }
};

class SendGoal : public BT::SyncActionNode
{
public:
  SendGoal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    geometry_msgs::PoseStamped goal;
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (!g_state.has_goal)
        return BT::NodeStatus::FAILURE;
      goal = g_state.goal;
    }
    goal.header.stamp = ros::Time::now();
    g_state.goal_pub.publish(goal);
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      g_state.last_event = "send_goal";
    }
    ROS_INFO_STREAM("[bt] send goal " << goal.pose.position.x << ", "
                                      << goal.pose.position.y << ", "
                                      << goal.pose.position.z);
    return BT::NodeStatus::SUCCESS;
  }
};

class WaitArrival : public BT::StatefulActionNode
{
public:
  WaitArrival(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), distance_(0.25) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("distance", 0.25, "arrival radius (m)"),
            BT::InputPort<double>("timeout", 60.0, "timeout (s)")};
  }

  BT::NodeStatus onStart() override
  {
    double distance = 0.25;
    double timeout = 60.0;
    getInput("distance", distance);
    getInput("timeout", timeout);
    distance_ = distance;
    g_timeout_s = timeout;
    g_start = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      g_state.last_event = "arrival_start";
    }
    return check();
  }

  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}

private:
  BT::NodeStatus check()
  {
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped pose;
    bool has_pose = false;
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      goal = g_state.goal;
      pose = g_state.pose;
      has_pose = g_state.has_pose;
    }
    if (has_pose)
    {
      const double dx = pose.pose.position.x - goal.pose.position.x;
      const double dy = pose.pose.position.y - goal.pose.position.y;
      if (std::hypot(dx, dy) <= distance_)
      {
        ROS_INFO("[bt] goal reached (%.3f m)", std::hypot(dx, dy));
        {
          std::lock_guard<std::mutex> lock(g_state.mutex);
          g_state.last_event = "reached";
        }
        return BT::NodeStatus::SUCCESS;
      }
    }
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - g_start).count();
    if (elapsed > g_timeout_s)
    {
      ROS_WARN("[bt] goal timeout after %.1f s", elapsed);
      {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        g_state.last_event = "timeout";
      }
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  double distance_;
};

class Stop : public BT::SyncActionNode
{
public:
  Stop(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    geometry_msgs::Twist zero;
    g_state.stop_pub.publish(zero);
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      g_state.last_event = "stop";
    }
    ROS_INFO("[bt] stop (zero teleop app command)");
    return BT::NodeStatus::SUCCESS;
  }
};

std::string statusName(BT::NodeStatus status)
{
  switch (status)
  {
    case BT::NodeStatus::IDLE: return "IDLE";
    case BT::NodeStatus::RUNNING: return "RUNNING";
    case BT::NodeStatus::SUCCESS: return "SUCCESS";
    case BT::NodeStatus::FAILURE: return "FAILURE";
  }
  return "UNKNOWN";
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(g_state.mutex);
  g_state.goal = *msg;
  g_state.goal.header.frame_id =
      msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
  g_state.has_goal = true;
  g_state.goal_generation++;
  ROS_INFO("[bt] new goal received");
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(g_state.mutex);
  g_state.pose = *msg;
  g_state.has_pose = true;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bt_console");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string tree_file;
  pnh.param<std::string>("tree_file", tree_file, "");
  if (tree_file.empty())
  {
    ROS_ERROR("~tree_file is required");
    return 1;
  }

  std::string goal_topic = "/v8/bt/goal";
  std::string pose_topic = "/v8/local/pose";
  std::string out_goal_topic = "/v8/3d/global/goal";
  std::string stop_topic = "/lite3/teleop/app_cmd_vel";
  pnh.param<std::string>("goal_topic", goal_topic, goal_topic);
  pnh.param<std::string>("pose_topic", pose_topic, pose_topic);
  pnh.param<std::string>("out_goal_topic", out_goal_topic, out_goal_topic);
  pnh.param<std::string>("stop_topic", stop_topic, stop_topic);

  g_state.goal_pub = nh.advertise<geometry_msgs::PoseStamped>(out_goal_topic, 1);
  g_state.stop_pub = nh.advertise<geometry_msgs::Twist>(stop_topic, 1);
  ros::Publisher status_pub = nh.advertise<std_msgs::String>("/v8/bt/status", 1);
  std::string lease_topic = "/v8/bt/lease";
  pnh.param<std::string>("lease_topic", lease_topic, lease_topic);
  ros::Publisher lease_pub = nh.advertise<std_msgs::Bool>(lease_topic, 1);

  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      goal_topic, 1, goalCallback);

  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      pose_topic, 1, poseCallback);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<WaitForGoal>("WaitForGoal");
  factory.registerNodeType<SendGoal>("SendGoal");
  factory.registerNodeType<WaitArrival>("WaitArrival");
  factory.registerNodeType<Stop>("Stop");

  BT::Tree tree;
  try
  {
    tree = factory.createTreeFromFile(tree_file);
  }
  catch (const std::exception& error)
  {
    ROS_ERROR_STREAM("failed to load tree " << tree_file << ": " << error.what());
    return 1;
  }

  BT::PublisherZMQ groot_publisher(tree, 25, 5555, 5556);
  ROS_INFO("[bt] tree loaded: %s", tree_file.c_str());
  ROS_INFO("[bt] Groot2 ZMQ publisher ports 5555/5556");

  ros::Rate loop_rate(20.0);
  ros::Time last_status(0.0);
  while (ros::ok())
  {
    tree.tickRoot();
    ros::spinOnce();

    if ((ros::Time::now() - last_status).toSec() >= 0.5)
    {
      std_msgs::Bool lease;
      lease.data = true;
      lease_pub.publish(lease);
    }
    if ((ros::Time::now() - last_status).toSec() >= 0.5)
    {
      std_msgs::String status;
      std::string event;
      {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        event = g_state.last_event;
      }
      std::string nodes = "[";
      for (size_t i = 0; i < tree.nodes.size(); ++i)
      {
        if (i) nodes += ",";
        nodes += std::string("{\"name\":\"") + tree.nodes[i]->name() +
                 "\",\"status\":\"" + statusName(tree.nodes[i]->status()) + "\"}";
      }
      nodes += "]";
      status.data = std::string("{\"tree\":\"NavigateSingleGoal\",\"root\":\"") +
                    statusName(tree.rootNode()->status()) + "\",\"event\":\"" + event +
                    "\",\"nodes\":" + nodes + "}";
      status_pub.publish(status);
      last_status = ros::Time::now();
    }
    loop_rate.sleep();
  }
  return 0;
}
