#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <lite3_bt/goal_queue.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

// Nav2-style orchestration tree for the host-only demo plane.
//
// The tree decides WHEN to ask for a global path, WHEN to hand the goal to the
// downstream executor, WHEN to dwell and WHEN to recover.  It never computes a
// path, never writes a velocity command other than the zero stop, and never
// decides the planner's replanning cadence.  Those remain downstream concerns.
namespace
{
struct SharedState
{
  std::mutex mutex;
  // FIFO goals: a fast multi-goal submission must not collapse into one goal.
  lite3_bt::GoalQueue<geometry_msgs::PoseStamped> pending;
  geometry_msgs::PoseStamped current_goal;
  geometry_msgs::PoseStamped pose;
  bool has_current = false;
  bool has_pose = false;
  std::uint64_t goal_seq = 0;
  std::uint64_t current_seq = 0;

  // Global planner result (/global_path/status).
  bool path_status_seen = false;
  bool path_ok = false;
  int path_points = 0;
  std::string path_reason;

  // Optional downstream executor result ("<seq>:ok" / "<seq>:fail").
  bool planner_ack_enabled = false;
  bool planner_ack_seen = false;
  bool planner_ack_ok = false;
  std::uint64_t planner_ack_seq = 0;

  std::string last_event = "idle";
  ros::Publisher goal_pub;
  ros::Publisher planner_goal_pub;
  ros::Publisher stop_pub;
};

SharedState g_state;

bool transient_plan_reason(const std::string& reason)
{
  return reason.empty() || reason == "E_PLAN_RUNNING" ||
         reason == "E_PLAN_NOT_READY" || reason == "E_PLAN_CANCELLED_REQUESTED";
}

std::string pathState()
{
  if (!g_state.path_status_seen) return "idle";
  if (g_state.path_ok && g_state.path_points > 0) return "ready";
  if (!g_state.path_ok && !transient_plan_reason(g_state.path_reason)) return "failed";
  return "running";
}

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
    geometry_msgs::PoseStamped next;
    if (!g_state.pending.pop(next)) return BT::NodeStatus::RUNNING;
    g_state.current_goal = next;
    g_state.has_current = true;
    g_state.current_seq = ++g_state.goal_seq;
    g_state.path_status_seen = false;
    g_state.path_ok = false;
    g_state.path_points = 0;
    g_state.path_reason.clear();
    g_state.planner_ack_seen = false;
    g_state.planner_ack_ok = false;
    g_state.planner_ack_seq = 0;
    g_state.last_event = "goal";
    ROS_INFO("[bt] goal %lu dequeued (%zu waiting)",
             static_cast<unsigned long>(g_state.current_seq), g_state.pending.depth());
    return BT::NodeStatus::SUCCESS;
  }
};

// ComputePath equivalent: publish the goal to the planner's INPUT topic and
// invalidate any previously observed path.  The planner runs asynchronously.
class RequestPath : public BT::SyncActionNode
{
public:
  RequestPath(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    geometry_msgs::PoseStamped goal;
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (!g_state.has_current) return BT::NodeStatus::FAILURE;
      goal = g_state.current_goal;
      g_state.path_status_seen = false;
      g_state.path_ok = false;
      g_state.path_points = 0;
      g_state.path_reason.clear();
      g_state.last_event = "request_path";
    }
    goal.header.stamp = ros::Time::now();
    g_state.planner_goal_pub.publish(goal);
    ROS_INFO("[bt] global path requested for goal %lu",
             static_cast<unsigned long>(g_state.current_seq));
    return BT::NodeStatus::SUCCESS;
  }
};

// Wait for the planner to report a usable path.  E_PLAN_RUNNING and friends
// are transient states, not failures; a terminal refusal ends this goal.
class WaitForPath : public BT::StatefulActionNode
{
public:
  WaitForPath(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), timeout_(30.0) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout", 30.0, "path wait timeout (s)")};
  }

  BT::NodeStatus onStart() override
  {
    getInput("timeout", timeout_);
    start_ = std::chrono::steady_clock::now();
    return check();
  }

  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}

private:
  BT::NodeStatus check()
  {
    bool seen = false, ok = false;
    int points = 0;
    std::string reason;
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      seen = g_state.path_status_seen;
      ok = g_state.path_ok;
      points = g_state.path_points;
      reason = g_state.path_reason;
    }
    if (seen && ok && points > 0)
    {
      {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        g_state.last_event = "path_ready";
      }
      ROS_INFO("[bt] global path ready (%d points)", points);
      return BT::NodeStatus::SUCCESS;
    }
    if (seen && !ok && !transient_plan_reason(reason))
    {
      ROS_WARN("[bt] global path refused: %s", reason.c_str());
      {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        g_state.last_event = "path_failed";
      }
      return BT::NodeStatus::FAILURE;
    }
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_).count();
    if (elapsed > timeout_)
    {
      ROS_WARN("[bt] global path timeout after %.1f s", elapsed);
      {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        g_state.last_event = "path_timeout";
      }
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  double timeout_;
  std::chrono::steady_clock::time_point start_;
};

// FollowPath equivalent for this demo: hand the goal to the downstream
// executor through the goal gate.  Local trajectory / velocity stay downstream.
class FollowPath : public BT::SyncActionNode
{
public:
  FollowPath(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    geometry_msgs::PoseStamped goal;
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (!g_state.has_current) return BT::NodeStatus::FAILURE;
      goal = g_state.current_goal;
    }
    goal.header.stamp = ros::Time::now();
    g_state.goal_pub.publish(goal);
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      g_state.last_event = "follow_path";
    }
    ROS_INFO_STREAM("[bt] follow path -> dispatch goal " << goal.pose.position.x << ", "
                                                        << goal.pose.position.y << ", "
                                                        << goal.pose.position.z);
    return BT::NodeStatus::SUCCESS;
  }
};

class WaitArrival : public BT::StatefulActionNode
{
public:
  WaitArrival(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), distance_(0.25), timeout_(60.0) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("distance", 0.25, "arrival radius (m)"),
            BT::InputPort<double>("timeout", 60.0, "timeout (s)")};
  }

  BT::NodeStatus onStart() override
  {
    getInput("distance", distance_);
    getInput("timeout", timeout_);
    start_ = std::chrono::steady_clock::now();
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
    geometry_msgs::PoseStamped goal, pose;
    bool has_pose = false, ack_enabled = false, ack_seen = false, ack_ok = false;
    std::uint64_t ack_seq = 0, current_seq = 0;
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      goal = g_state.current_goal;
      pose = g_state.pose;
      has_pose = g_state.has_pose;
      ack_enabled = g_state.planner_ack_enabled;
      ack_seen = g_state.planner_ack_seen;
      ack_ok = g_state.planner_ack_ok;
      ack_seq = g_state.planner_ack_seq;
      current_seq = g_state.current_seq;
    }

    // Ack mode: the downstream executor owns the outcome; the BT does not
    // second-guess it with its own pose-distance guess.
    if (ack_enabled)
    {
      if (ack_seen && ack_seq == current_seq && !ack_ok)
      {
        {
          std::lock_guard<std::mutex> lock(g_state.mutex);
          g_state.last_event = "planner_fail";
        }
        return BT::NodeStatus::FAILURE;
      }
      if (lite3_bt::ack_advances(ack_seq, ack_ok, current_seq))
      {
        {
          std::lock_guard<std::mutex> lock(g_state.mutex);
          g_state.last_event = "reached";
        }
        return BT::NodeStatus::SUCCESS;
      }
    }
    else if (has_pose)
    {
      const double dx = pose.pose.position.x - goal.pose.position.x;
      const double dy = pose.pose.position.y - goal.pose.position.y;
      if (std::hypot(dx, dy) <= distance_)
      {
        {
          std::lock_guard<std::mutex> lock(g_state.mutex);
          g_state.last_event = "reached";
        }
        return BT::NodeStatus::SUCCESS;
      }
    }
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_).count();
    if (elapsed > timeout_)
    {
      {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        g_state.last_event = "timeout";
      }
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  double distance_;
  double timeout_;
  std::chrono::steady_clock::time_point start_;
};

// Nav2 "Wait" equivalent: a deliberate dwell between goals.
class Dwell : public BT::StatefulActionNode
{
public:
  Dwell(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), seconds_(1.0) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("seconds", 1.0, "dwell after arrival (s)")};
  }

  BT::NodeStatus onStart() override
  {
    getInput("seconds", seconds_);
    start_ = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      g_state.last_event = "dwell";
    }
    return check();
  }

  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}

private:
  BT::NodeStatus check()
  {
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_).count();
    return elapsed >= seconds_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  double seconds_;
  std::chrono::steady_clock::time_point start_;
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
    return BT::NodeStatus::SUCCESS;
  }
};

// Recovery branch: zero command and invalidate the stale path.  The failed
// goal is already dequeued, so the next tick moves on to the next goal.
class RecoveryStop : public BT::SyncActionNode
{
public:
  RecoveryStop(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    geometry_msgs::Twist zero;
    g_state.stop_pub.publish(zero);
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      g_state.path_status_seen = false;
      g_state.path_ok = false;
      g_state.path_points = 0;
      g_state.path_reason.clear();
      g_state.last_event = "recovered";
    }
    ROS_WARN("[bt] recovery: stop and invalidate path");
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
  geometry_msgs::PoseStamped goal = *msg;
  goal.header.frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
  std::lock_guard<std::mutex> lock(g_state.mutex);
  if (!g_state.pending.push(goal))
  {
    g_state.last_event = "queue_full";
    ROS_WARN("[bt] goal queue full (%zu); goal refused, none dropped",
             g_state.pending.max_size());
    return;
  }
  g_state.last_event = "queued";
  ROS_INFO("[bt] goal queued (depth=%zu, max=%zu)",
           g_state.pending.depth(), g_state.pending.max_size());
}

void pathStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  bool ok = false;
  int points = 0;
  std::string reason;
  if (!lite3_bt::parse_path_status(msg->data, &ok, &points, &reason))
  {
    ROS_WARN("[bt] malformed /global_path/status ignored");
    return;
  }
  std::lock_guard<std::mutex> lock(g_state.mutex);
  g_state.path_status_seen = true;
  g_state.path_ok = ok;
  g_state.path_points = points;
  g_state.path_reason = reason;
  g_state.last_event = (ok && points > 0) ? "path_ready"
                       : (!ok && !transient_plan_reason(reason)) ? "path_failed"
                       : "path_running";
}

void plannerAckCallback(const std_msgs::String::ConstPtr& msg)
{
  std::uint64_t seq = 0;
  bool ok = false;
  if (!lite3_bt::parse_planner_ack(msg->data, &seq, &ok))
  {
    ROS_WARN("[bt] malformed planner ack ignored: %s", msg->data.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(g_state.mutex);
  g_state.planner_ack_seq = seq;
  g_state.planner_ack_ok = ok;
  g_state.planner_ack_seen = true;
  g_state.last_event = ok ? "planner_ok" : "planner_fail";
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
  std::string out_goal_topic = "/v8/bt/goal_request";
  std::string stop_topic = "/lite3/teleop/app_cmd_vel";
  std::string planner_goal_topic = "/v8/global/goal";
  std::string path_status_topic = "/global_path/status";
  pnh.param<std::string>("goal_topic", goal_topic, goal_topic);
  pnh.param<std::string>("pose_topic", pose_topic, pose_topic);
  pnh.param<std::string>("out_goal_topic", out_goal_topic, out_goal_topic);
  pnh.param<std::string>("stop_topic", stop_topic, stop_topic);
  pnh.param<std::string>("planner_goal_topic", planner_goal_topic, planner_goal_topic);
  pnh.param<std::string>("path_status_topic", path_status_topic, path_status_topic);

  int queue_max = 16;
  pnh.param<int>("queue_max", queue_max, 16);
  g_state.pending.set_max(static_cast<std::size_t>(queue_max < 1 ? 1 : queue_max));

  // Optional downstream executor result.  Empty = pose-based arrival.
  std::string planner_result_topic;
  pnh.param<std::string>("planner_result_topic", planner_result_topic, std::string());
  g_state.planner_ack_enabled = !planner_result_topic.empty();

  g_state.goal_pub = nh.advertise<geometry_msgs::PoseStamped>(out_goal_topic, 1);
  g_state.planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>(planner_goal_topic, 1);
  g_state.stop_pub = nh.advertise<geometry_msgs::Twist>(stop_topic, 1);
  ros::Publisher status_pub = nh.advertise<std_msgs::String>("/v8/bt/status", 1);
  std::string lease_topic = "/v8/bt/lease";
  pnh.param<std::string>("lease_topic", lease_topic, lease_topic);
  ros::Publisher lease_pub = nh.advertise<std_msgs::Bool>(lease_topic, 1);

  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      goal_topic, 1, goalCallback);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      pose_topic, 1, poseCallback);
  ros::Subscriber path_status_sub = nh.subscribe<std_msgs::String>(
      path_status_topic, 10, pathStatusCallback);
  ros::Subscriber planner_ack_sub;
  if (g_state.planner_ack_enabled)
  {
    planner_ack_sub = nh.subscribe<std_msgs::String>(
        planner_result_topic, 1, plannerAckCallback);
    ROS_INFO("[bt] executor ack mode: %s", planner_result_topic.c_str());
  }
  else
  {
    ROS_INFO("[bt] pose-based arrival mode (no planner_result_topic)");
  }

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<WaitForGoal>("WaitForGoal");
  factory.registerNodeType<RequestPath>("RequestPath");
  factory.registerNodeType<WaitForPath>("WaitForPath");
  factory.registerNodeType<FollowPath>("FollowPath");
  // Legacy alias so navigate_single_goal.xml still loads.
  factory.registerNodeType<FollowPath>("SendGoal");
  factory.registerNodeType<WaitArrival>("WaitArrival");
  factory.registerNodeType<Dwell>("Dwell");
  factory.registerNodeType<Stop>("Stop");
  factory.registerNodeType<RecoveryStop>("RecoveryStop");

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
  ROS_INFO("[bt] planner input %s, status %s", planner_goal_topic.c_str(),
           path_status_topic.c_str());

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
      std::string event, path_state;
      std::size_t depth = 0;
      int path_points = 0;
      std::uint64_t goal_seq = 0, current_seq = 0;
      const char* planner_mode = "pose";
      {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        event = g_state.last_event;
        path_state = pathState();
        path_points = g_state.path_points;
        depth = g_state.pending.depth();
        goal_seq = g_state.goal_seq;
        current_seq = g_state.current_seq;
        planner_mode = g_state.planner_ack_enabled ? "ack" : "pose";
      }
      std::string nodes = "[";
      for (size_t i = 0; i < tree.nodes.size(); ++i)
      {
        if (i) nodes += ",";
        nodes += std::string("{\"name\":\"") + tree.nodes[i]->name() +
                 "\",\"status\":\"" + statusName(tree.nodes[i]->status()) + "\"}";
      }
      nodes += "]";
      status.data = std::string("{\"tree\":\"NavigateWithPlanner\",\"root\":\"") +
                    statusName(tree.rootNode()->status()) + "\",\"event\":\"" + event +
                    "\",\"queue_depth\":" + std::to_string(depth) +
                    ",\"goal_seq\":" + std::to_string(goal_seq) +
                    ",\"current_seq\":" + std::to_string(current_seq) +
                    ",\"path_state\":\"" + path_state + "\"" +
                    ",\"path_points\":" + std::to_string(path_points) +
                    ",\"planner_mode\":\"" + planner_mode + "\"" +
                    ",\"nodes\":" + nodes + "}";
      status_pub.publish(status);
      last_status = ros::Time::now();
    }
    loop_rate.sleep();
  }
  return 0;
}
