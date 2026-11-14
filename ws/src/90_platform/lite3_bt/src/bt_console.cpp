#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <lite3_bt/arrival_budget.hpp>
#include <lite3_bt/goal_queue.hpp>
#include <lite3_bt/bt_policy.hpp>

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
struct GoalItem
{
  geometry_msgs::PoseStamped goal;
  lite3_bt::GoalPolicy policy;
};

struct SharedState
{
  std::mutex mutex;
  // FIFO goals: a fast multi-goal submission must not collapse into one goal.
  lite3_bt::GoalQueue<GoalItem> pending;
  geometry_msgs::PoseStamped current_goal;
  lite3_bt::GoalPolicy current_policy;
  lite3_bt::GoalPolicy next_policy;
  geometry_msgs::PoseStamped pose;
  bool has_current = false;
  bool has_pose = false;
  std::uint64_t goal_seq = 0;
  std::uint64_t current_seq = 0;
  int attempts = 0;
  int goal_index = 0;
  int goals_total = 0;
  bool retry_queued = false;

  // Operator cancel: latched until a new goal or CancelStop completion.
  bool cancel_requested = false;
  std::string cancel_scope = "current";
  std::string cancel_reason;
  std::uint64_t cancel_seq = 0;
  bool cancel_ack_seen = false;
  bool cancel_ack_ok = false;
  std::uint64_t cancel_ack_seq = 0;
  std::string cancel_ack_outcome;
  double cancel_ack_timeout = 2.0;

  // Localization readiness (disabled when the topic is empty).
  bool localization_check = false;
  bool localization_ok = true;

  // Recovery policy (launch defaults; per-goal retry_max overrides).
  std::string recovery_mode = "skip";
  int retry_max = 1;
  double retry_delay = 0.0;

  // Global planner result (/global_path/status).
  bool path_status_seen = false;
  bool path_ok = false;
  int path_points = 0;
  std::string path_reason;

  // Latest /global_path geometry.  Point count cannot tell a 0.5 m goal from a
  // 7.5 m ramp; the polyline length is what sizes the arrival budget.
  bool path_length_seen = false;
  double path_length = 0.0;

  // Wall-clock progress floor for arrival budget scaling (pnh param).
  double arrival_speed_floor = 0.02;

  // Optional downstream executor result ("<seq>:ok" / "<seq>:fail").
  bool planner_ack_enabled = false;
  bool planner_ack_seen = false;
  bool planner_ack_ok = false;
  std::uint64_t planner_ack_seq = 0;

  // Optional external arrival signal (std_msgs/Bool), e.g. SCAN's
  // /lite3/scan/arrived.  Reset when a new goal is dequeued so a stale true
  // cannot short-circuit the next leg.
  bool arrival_signal_enabled = false;
  bool arrival_signal = false;

  std::string last_event = "idle";
  // Persistent outcome of the current goal.  last_event is overwritten by
  // dwell/stop within a single tick, so a 2 Hz status subscriber can never
  // observe "reached".  This survives until WaitForGoal dequeues a new goal.
  std::string goal_outcome;
  std::uint64_t outcome_seq = 0;
  ros::Publisher goal_pub;
  ros::Publisher planner_goal_pub;
  ros::Publisher cancel_pub;
  ros::Publisher stop_pub;
};

SharedState g_state;

// Caller must hold g_state.mutex.  Records the terminal result once so the
// status publisher can report it on every subsequent 0.5 s tick.
void recordOutcomeLocked(bool ok)
{
  g_state.goal_outcome = ok ? "reached" : "failed";
  g_state.outcome_seq = g_state.current_seq;
}

void recordCancelOutcomeLocked(const std::string& outcome)
{
  g_state.goal_outcome = outcome;
  g_state.outcome_seq = g_state.current_seq;
}

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
    if (g_state.cancel_requested)
    {
      g_state.last_event = "cancel_requested";
      return BT::NodeStatus::FAILURE;
    }
    GoalItem next;
    if (!g_state.pending.pop(next)) return BT::NodeStatus::RUNNING;
    g_state.current_goal = next.goal;
    g_state.current_policy = next.policy;
    g_state.has_current = true;
    g_state.current_seq = ++g_state.goal_seq;
    if (!g_state.retry_queued)
    {
      g_state.attempts = 0;
      g_state.goal_index = next.policy.goal_index > 0 ? next.policy.goal_index
                                                      : g_state.goal_index + 1;
      g_state.goals_total = next.policy.goals_total;
    }
    g_state.retry_queued = false;
    g_state.cancel_requested = false;
    g_state.cancel_ack_seen = false;
    g_state.path_status_seen = false;
    g_state.path_ok = false;
    g_state.path_points = 0;
    g_state.path_reason.clear();
    g_state.planner_ack_seen = false;
    g_state.planner_ack_ok = false;
    g_state.planner_ack_seq = 0;
    g_state.arrival_signal = false;
    g_state.goal_outcome.clear();
    g_state.outcome_seq = 0;
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
      if (g_state.cancel_requested)
      {
        g_state.last_event = "cancel_requested";
        return BT::NodeStatus::FAILURE;
      }
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
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (g_state.current_policy.path_timeout_s > 0.0)
        timeout_ = g_state.current_policy.path_timeout_s;
    }
    start_ = std::chrono::steady_clock::now();
    return check();
  }

  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}

private:
  BT::NodeStatus check()
  {
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (g_state.cancel_requested)
      {
        g_state.last_event = "cancel_requested";
        return BT::NodeStatus::FAILURE;
      }
    }
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
        recordOutcomeLocked(false);
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
        recordOutcomeLocked(false);
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
      if (g_state.cancel_requested)
      {
        g_state.last_event = "cancel_requested";
        return BT::NodeStatus::FAILURE;
      }
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
    : BT::StatefulActionNode(name, config), distance_(0.25), timeout_(60.0),
      budget_(60.0) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("distance", 0.25, "arrival radius (m)"),
            BT::InputPort<double>("timeout", 60.0, "timeout (s)")};
  }

  BT::NodeStatus onStart() override
  {
    getInput("distance", distance_);
    getInput("timeout", timeout_);
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (g_state.current_policy.arrival_timeout_s > 0.0)
        timeout_ = g_state.current_policy.arrival_timeout_s;
    }
    double length = 0.0;
    bool seen = false;
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      length = g_state.path_length;
      seen = g_state.path_length_seen;
      g_state.last_event = "arrival_start";
    }
    // Freeze the budget once per arrival: the planner replans from the moving
    // start, so the remaining path shrinks while we drive and a per-tick
    // recompute would shrink the budget below the elapsed time mid-leg.
    budget_ = lite3_bt::arrival_budget_s(timeout_, seen ? length : 0.0,
                                         g_state.arrival_speed_floor);
    if (budget_ > timeout_)
    {
      ROS_INFO("[bt] arrival budget %.1f s (path %.2f m, floor %.3f m/s, tree %.1f s)",
               budget_, length, g_state.arrival_speed_floor, timeout_);
    }
    start_ = std::chrono::steady_clock::now();
    return check();
  }

  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}

private:
  BT::NodeStatus check()
  {
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (g_state.cancel_requested)
      {
        g_state.last_event = "cancel_requested";
        return BT::NodeStatus::FAILURE;
      }
    }
    geometry_msgs::PoseStamped goal, pose;
    bool has_pose = false, ack_enabled = false, ack_seen = false, ack_ok = false;
    bool signal_enabled = false, signal_arrived = false;
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
      signal_enabled = g_state.arrival_signal_enabled;
      signal_arrived = g_state.arrival_signal;
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
          recordOutcomeLocked(false);
        }
        return BT::NodeStatus::FAILURE;
      }
      if (lite3_bt::ack_advances(ack_seq, ack_ok, current_seq))
      {
        {
          std::lock_guard<std::mutex> lock(g_state.mutex);
          g_state.last_event = "reached";
          recordOutcomeLocked(true);
        }
        return BT::NodeStatus::SUCCESS;
      }
    }
    else if (signal_enabled)
    {
      // External arrival owner (e.g. SCAN's /lite3/scan/arrived).  The signal
      // only asserts true, so it can never fail the goal on its own; the
      // budget below stays the silent-executor guard.
      if (signal_arrived)
      {
        {
          std::lock_guard<std::mutex> lock(g_state.mutex);
          g_state.last_event = "reached";
          recordOutcomeLocked(true);
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
          recordOutcomeLocked(true);
        }
        return BT::NodeStatus::SUCCESS;
      }
    }
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_).count();
    if (elapsed > budget_)
    {
      ROS_WARN("[bt] arrival timeout after %.1f s (budget %.1f s)", elapsed, budget_);
      {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        g_state.last_event = "timeout";
        recordOutcomeLocked(false);
      }
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  double distance_;
  double timeout_;
  double budget_;
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
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (g_state.current_policy.dwell_s > 0.0) seconds_ = g_state.current_policy.dwell_s;
    }
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
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (g_state.cancel_requested)
      {
        g_state.last_event = "cancel_requested";
        return BT::NodeStatus::FAILURE;
      }
    }
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

// Cancel is an operator request, not a velocity hint.  CancelStop publishes the
// goal-cancel to the executor, publishes a zero command as a last resort, and
// waits for the executor ack.  An unacked cancel is reported, never silently
// treated as success; the safety mux still stops independently.
class CancelStop : public BT::StatefulActionNode
{
public:
  CancelStop(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override
  {
    std::uint64_t seq = 0;
    std::string scope, reason;
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (!g_state.cancel_requested) return BT::NodeStatus::FAILURE;
      seq = g_state.cancel_seq;
      scope = g_state.cancel_scope;
      reason = g_state.cancel_reason;
      g_state.has_current = false;
      g_state.last_event = "cancelling";
      if (scope == "all") g_state.pending.clear();
    }
    std_msgs::String command;
    command.data = std::to_string(seq) + ":" + scope + ":" + reason;
    g_state.cancel_pub.publish(command);
    geometry_msgs::Twist zero;
    g_state.stop_pub.publish(zero);
    start_ = std::chrono::steady_clock::now();
    return check(seq);
  }

  BT::NodeStatus onRunning() override { return check(seq_); }
  void onHalted() override {}

private:
  BT::NodeStatus check(std::uint64_t seq)
  {
    seq_ = seq;
    std::string outcome;
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (g_state.cancel_ack_seen && g_state.cancel_ack_seq == seq)
        outcome = g_state.cancel_ack_outcome;
      else if (std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - start_).count()
               > g_state.cancel_ack_timeout)
        outcome = "unacked";
    }
    if (outcome.empty()) return BT::NodeStatus::RUNNING;
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      g_state.cancel_requested = false;
      g_state.cancel_ack_seen = false;
      g_state.last_event = outcome == "cancelled" ? "cancelled"
                           : outcome == "ack_fail" ? "cancel_ack_fail"
                                                   : "cancel_unacked";
      recordCancelOutcomeLocked(outcome == "cancelled" ? "cancelled" : g_state.last_event);
    }
    return outcome == "cancelled" ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  std::uint64_t seq_ = 0;
  std::chrono::steady_clock::time_point start_;
};

class IsCancelled : public BT::ConditionNode
{
public:
  IsCancelled(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(g_state.mutex);
    return g_state.cancel_requested ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsNotCancelled : public BT::ConditionNode
{
public:
  IsNotCancelled(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(g_state.mutex);
    return g_state.cancel_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

class CheckLocalization : public BT::SyncActionNode
{
public:
  CheckLocalization(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(g_state.mutex);
    if (!g_state.localization_check || g_state.localization_ok)
      return BT::NodeStatus::SUCCESS;
    g_state.last_event = "localization_lost";
    recordOutcomeLocked(false);
    return BT::NodeStatus::FAILURE;
  }
};

// Configurable recovery: retry/replan re-queue the failed goal at the head,
// skip moves on, abort clears the queue.  Retry budget is per goal.
class Recovery : public BT::StatefulActionNode
{
public:
  Recovery(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override
  {
    {
      std::lock_guard<std::mutex> lock(g_state.mutex);
      if (!g_state.has_current) return BT::NodeStatus::SUCCESS;
      const int budget = g_state.current_policy.retry_max >= 0
          ? g_state.current_policy.retry_max : g_state.retry_max;
      decision_ = lite3_bt::decide_recovery(g_state.recovery_mode, g_state.attempts,
                                            budget, "skip");
      goal_ = g_state.current_goal;
      policy_ = g_state.current_policy;
      delay_ = g_state.retry_delay;
      start_ = std::chrono::steady_clock::now();
      if (decision_.action != lite3_bt::RecoveryAction::RETRY || delay_ <= 0.0)
        return applyLocked();
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_).count();
    if (elapsed < delay_) return BT::NodeStatus::RUNNING;
    std::lock_guard<std::mutex> lock(g_state.mutex);
    return applyLocked();
  }

  void onHalted() override {}

private:
  BT::NodeStatus applyLocked()
  {
    if (decision_.action == lite3_bt::RecoveryAction::RETRY)
    {
      GoalItem item;
      item.goal = goal_;
      item.policy = policy_;
      if (g_state.pending.push_front(item))
      {
        g_state.attempts += 1;
        g_state.retry_queued = true;
        g_state.has_current = false;
        g_state.last_event = g_state.recovery_mode == "replan" ? "replan" : "retry";
        g_state.path_status_seen = false;
        g_state.path_ok = false;
        g_state.path_points = 0;
        g_state.path_reason.clear();
        return BT::NodeStatus::SUCCESS;
      }
    }
    geometry_msgs::Twist zero;
    g_state.stop_pub.publish(zero);
    if (decision_.action == lite3_bt::RecoveryAction::ABORT)
    {
      g_state.pending.clear();
      g_state.last_event = "recovered_abort";
    }
    else
    {
      g_state.last_event = decision_.exhausted ? "retry_exhausted" : "recovered_skip";
    }
    if (g_state.goal_outcome.empty() || decision_.exhausted)
      recordOutcomeLocked(false);
    g_state.has_current = false;
    g_state.path_status_seen = false;
    g_state.path_ok = false;
    g_state.path_points = 0;
    g_state.path_reason.clear();
    return BT::NodeStatus::SUCCESS;
  }

  lite3_bt::RecoveryDecision decision_{lite3_bt::RecoveryAction::SKIP, false};
  geometry_msgs::PoseStamped goal_;
  lite3_bt::GoalPolicy policy_;
  double delay_ = 0.0;
  std::chrono::steady_clock::time_point start_;
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
  GoalItem item;
  item.goal = *msg;
  item.goal.header.frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
  std::lock_guard<std::mutex> lock(g_state.mutex);
  item.policy = g_state.next_policy;
  g_state.next_policy = lite3_bt::GoalPolicy();
  g_state.cancel_requested = false;
  if (!g_state.pending.push(item))
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

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  double length = 0.0;
  for (std::size_t i = 1; i < msg->poses.size(); ++i)
  {
    const auto& a = msg->poses[i - 1].pose.position;
    const auto& b = msg->poses[i].pose.position;
    length += std::sqrt(std::pow(b.x - a.x, 2.0) + std::pow(b.y - a.y, 2.0) +
                        std::pow(b.z - a.z, 2.0));
  }
  std::lock_guard<std::mutex> lock(g_state.mutex);
  // An empty Path is the planner's refusal/cancel marker; it must not lend its
  // length to the next arrival.
  g_state.path_length_seen = msg->poses.size() >= 2;
  g_state.path_length = g_state.path_length_seen ? length : 0.0;
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

void arrivalCallback(const std_msgs::Bool::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(g_state.mutex);
  g_state.arrival_signal = msg->data;
  g_state.last_event = msg->data ? "arrived_signal" : "arrival_reset";
}

void missionCallback(const std_msgs::String::ConstPtr& msg)
{
  double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
  lite3_bt::GoalPolicy policy;
  if (!lite3_bt::parse_mission_goal(msg->data, &x, &y, &z, &yaw, &policy))
  {
    ROS_WARN("[bt] malformed mission goal ignored: %s", msg->data.c_str());
    return;
  }
  GoalItem item;
  item.goal.header.frame_id = "map";
  item.goal.header.stamp = ros::Time::now();
  item.goal.pose.position.x = x;
  item.goal.pose.position.y = y;
  item.goal.pose.position.z = z;
  item.goal.pose.orientation.z = std::sin(yaw / 2.0);
  item.goal.pose.orientation.w = std::cos(yaw / 2.0);
  item.policy = policy;
  std::lock_guard<std::mutex> lock(g_state.mutex);
  g_state.cancel_requested = false;
  if (!g_state.pending.push(item))
  {
    g_state.last_event = "queue_full";
    ROS_WARN("[bt] mission queue full (%zu); goal refused, none dropped",
             g_state.pending.max_size());
    return;
  }
  g_state.last_event = "queued";
  ROS_INFO("[bt] mission goal %d/%d queued (depth=%zu)", policy.goal_index,
           policy.goals_total, g_state.pending.depth());
}

void policyCallback(const std_msgs::String::ConstPtr& msg)
{
  lite3_bt::GoalPolicy policy;
  if (!lite3_bt::parse_goal_policy(msg->data, &policy))
  {
    ROS_WARN("[bt] malformed goal policy ignored: %s", msg->data.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(g_state.mutex);
  g_state.next_policy = policy;
  g_state.last_event = "policy";
}

void cancelCallback(const std_msgs::String::ConstPtr& msg)
{
  std::uint64_t seq = 0;
  std::string scope, reason;
  if (!lite3_bt::parse_cancel(msg->data, &seq, &scope, &reason))
  {
    ROS_WARN("[bt] malformed cancel ignored: %s", msg->data.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(g_state.mutex);
  g_state.cancel_requested = true;
  g_state.cancel_seq = seq;
  g_state.cancel_scope = scope;
  g_state.cancel_reason = reason;
  g_state.cancel_ack_seen = false;
  g_state.cancel_ack_outcome.clear();
  if (scope == "all") g_state.pending.clear();
  g_state.last_event = "cancel_requested";
  ROS_WARN("[bt] cancel requested seq=%lu scope=%s", static_cast<unsigned long>(seq),
           scope.c_str());
}

void cancelAckCallback(const std_msgs::String::ConstPtr& msg)
{
  std::uint64_t seq = 0;
  std::string outcome;
  if (!lite3_bt::parse_cancel_ack(msg->data, &seq, &outcome))
  {
    ROS_WARN("[bt] malformed cancel ack ignored: %s", msg->data.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(g_state.mutex);
  g_state.cancel_ack_seen = true;
  g_state.cancel_ack_seq = seq;
  g_state.cancel_ack_outcome = outcome;
  g_state.cancel_ack_ok = outcome == "cancelled";
}

void localizationCallback(const std_msgs::Bool::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(g_state.mutex);
  g_state.localization_ok = msg->data;
  g_state.last_event = msg->data ? "localization_ok" : "localization_lost";
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
  std::string path_topic = "/global_path";
  pnh.param<std::string>("goal_topic", goal_topic, goal_topic);
  pnh.param<std::string>("pose_topic", pose_topic, pose_topic);
  pnh.param<std::string>("out_goal_topic", out_goal_topic, out_goal_topic);
  pnh.param<std::string>("stop_topic", stop_topic, stop_topic);
  pnh.param<std::string>("planner_goal_topic", planner_goal_topic, planner_goal_topic);
  pnh.param<std::string>("path_status_topic", path_status_topic, path_status_topic);
  pnh.param<std::string>("path_topic", path_topic, path_topic);
  pnh.param<double>("arrival_speed_floor_mps", g_state.arrival_speed_floor,
                    g_state.arrival_speed_floor);

  int queue_max = 16;
  pnh.param<int>("queue_max", queue_max, 16);
  g_state.pending.set_max(static_cast<std::size_t>(queue_max < 1 ? 1 : queue_max));

  pnh.param<double>("cancel_ack_timeout", g_state.cancel_ack_timeout, g_state.cancel_ack_timeout);
  pnh.param<std::string>("recovery_mode", g_state.recovery_mode, g_state.recovery_mode);
  pnh.param<int>("retry_max", g_state.retry_max, g_state.retry_max);
  pnh.param<double>("retry_delay_s", g_state.retry_delay, g_state.retry_delay);
  if (g_state.recovery_mode != "skip" && g_state.recovery_mode != "retry"
      && g_state.recovery_mode != "replan" && g_state.recovery_mode != "abort")
  {
    ROS_ERROR("~recovery_mode must be skip|retry|replan|abort");
    return 1;
  }
  std::string cancel_input_topic = "/v8/bt/cancel";
  std::string cancel_topic = "/v8/local/cancel";
  std::string cancel_ack_topic = "/v8/local/cancel_ack";
  std::string policy_topic = "/v8/bt/goal_policy";
  std::string mission_topic = "/v8/bt/goal_mission";
  std::string localization_topic;
  pnh.param<std::string>("cancel_input_topic", cancel_input_topic, cancel_input_topic);
  pnh.param<std::string>("cancel_topic", cancel_topic, cancel_topic);
  pnh.param<std::string>("cancel_ack_topic", cancel_ack_topic, cancel_ack_topic);
  pnh.param<std::string>("policy_topic", policy_topic, policy_topic);
  pnh.param<std::string>("mission_topic", mission_topic, mission_topic);
  pnh.param<std::string>("localization_topic", localization_topic, std::string());

  // Optional downstream executor result.  Empty = pose-based arrival.
  std::string planner_result_topic;
  pnh.param<std::string>("planner_result_topic", planner_result_topic, std::string());
  g_state.planner_ack_enabled = !planner_result_topic.empty();
  std::string arrival_topic;
  pnh.param<std::string>("arrival_topic", arrival_topic, std::string());
  g_state.arrival_signal_enabled = !arrival_topic.empty();
  if (g_state.planner_ack_enabled && g_state.arrival_signal_enabled)
  {
    ROS_ERROR("~planner_result_topic and ~arrival_topic are mutually exclusive");
    return 1;
  }

  g_state.goal_pub = nh.advertise<geometry_msgs::PoseStamped>(out_goal_topic, 1);
  g_state.planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>(planner_goal_topic, 1);
  g_state.stop_pub = nh.advertise<geometry_msgs::Twist>(stop_topic, 1);
  g_state.cancel_pub = nh.advertise<std_msgs::String>(cancel_topic, 1);
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
  ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>(
      path_topic, 1, pathCallback);
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
  ros::Subscriber arrival_sub;
  if (g_state.arrival_signal_enabled)
  {
    arrival_sub = nh.subscribe<std_msgs::Bool>(arrival_topic, 1, arrivalCallback);
    ROS_INFO("[bt] external arrival signal mode: %s", arrival_topic.c_str());
  }
  ros::Subscriber cancel_sub = nh.subscribe<std_msgs::String>(
      cancel_input_topic, 1, cancelCallback);
  ros::Subscriber cancel_ack_sub = nh.subscribe<std_msgs::String>(
      cancel_ack_topic, 1, cancelAckCallback);
  ros::Subscriber policy_sub = nh.subscribe<std_msgs::String>(
      policy_topic, 1, policyCallback);
  ros::Subscriber mission_sub = nh.subscribe<std_msgs::String>(
      mission_topic, 1, missionCallback);
  ros::Subscriber localization_sub;
  if (!localization_topic.empty())
  {
    g_state.localization_check = true;
    localization_sub = nh.subscribe<std_msgs::Bool>(
        localization_topic, 1, localizationCallback);
    ROS_INFO("[bt] localization gate: %s", localization_topic.c_str());
  }
  ROS_INFO("[bt] cancel in %s, out %s, ack in %s, policy in %s, mission in %s",
           cancel_input_topic.c_str(), cancel_topic.c_str(),
           cancel_ack_topic.c_str(), policy_topic.c_str(), mission_topic.c_str());

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
  factory.registerNodeType<Recovery>("Recovery");
  factory.registerNodeType<CancelStop>("CancelStop");
  factory.registerNodeType<IsCancelled>("IsCancelled");
  factory.registerNodeType<IsNotCancelled>("IsNotCancelled");
  factory.registerNodeType<CheckLocalization>("CheckLocalization");

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

  // ZMQ ports were hard-coded, so a second instance on the same host (another
  // master, a parallel test run, or sim + robot on one box) died in
  // PublisherZMQ on bind.  Defaults keep the previous behaviour.
  int zmq_pub_port = 5555;
  int zmq_sub_port = 5556;
  pnh.param<int>("zmq_pub_port", zmq_pub_port, 5555);
  pnh.param<int>("zmq_sub_port", zmq_sub_port, 5556);
  BT::PublisherZMQ groot_publisher(tree, 25,
                                   static_cast<unsigned>(zmq_pub_port),
                                   static_cast<unsigned>(zmq_sub_port));
  ROS_INFO("[bt] tree loaded: %s", tree_file.c_str());
  ROS_INFO("[bt] planner input %s, status %s", planner_goal_topic.c_str(),
           path_status_topic.c_str());
  ROS_INFO("[bt] Groot2 ZMQ publisher ports %d/%d", zmq_pub_port, zmq_sub_port);

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
      std::string event, path_state, outcome;
      std::size_t depth = 0;
      int path_points = 0;
      std::uint64_t goal_seq = 0, current_seq = 0, outcome_seq = 0;
      bool arrival_signal = false;
      const char* planner_mode = "pose";
      bool cancel_requested = false;
      std::string cancel_scope, cancel_ack, recovery_mode;
      int attempts = 0, goal_index = 0, goals_total = 0;
      bool localization_ok = true, localization_check = false;
      {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        event = g_state.last_event;
        outcome = g_state.goal_outcome;
        outcome_seq = g_state.outcome_seq;
        path_state = pathState();
        path_points = g_state.path_points;
        depth = g_state.pending.depth();
        goal_seq = g_state.goal_seq;
        current_seq = g_state.current_seq;
        arrival_signal = g_state.arrival_signal;
        planner_mode = g_state.planner_ack_enabled ? "ack"
                       : g_state.arrival_signal_enabled ? "signal" : "pose";
        cancel_requested = g_state.cancel_requested;
        cancel_scope = g_state.cancel_scope;
        cancel_ack = g_state.cancel_ack_outcome;
        recovery_mode = g_state.recovery_mode;
        attempts = g_state.attempts;
        goal_index = g_state.goal_index;
        goals_total = g_state.goals_total;
        localization_ok = g_state.localization_ok;
        localization_check = g_state.localization_check;
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
                    ",\"outcome\":\"" + outcome + "\"" +
                    ",\"outcome_seq\":" + std::to_string(outcome_seq) +
                    ",\"path_state\":\"" + path_state + "\"" +
                    ",\"path_points\":" + std::to_string(path_points) +
                    ",\"planner_mode\":\"" + planner_mode + "\"" +
                    ",\"cancel_requested\":" + std::string(cancel_requested ? "true" : "false") +
                    ",\"cancel_scope\":\"" + cancel_scope + "\"" +
                    ",\"cancel_ack\":\"" + cancel_ack + "\"" +
                    ",\"attempts\":" + std::to_string(attempts) +
                    ",\"recovery_mode\":\"" + recovery_mode + "\"" +
                    ",\"goal_index\":" + std::to_string(goal_index) +
                    ",\"goals_total\":" + std::to_string(goals_total) +
                    ",\"localization_ok\":" + std::string(localization_ok ? "true" : "false") +
                    ",\"localization_check\":" + std::string(localization_check ? "true" : "false") +
                    ",\"arrival_signal\":" + std::string(arrival_signal ? "true" : "false") +
                    ",\"nodes\":" + nodes + "}";
      status_pub.publish(status);
      last_status = ros::Time::now();
    }
    loop_rate.sleep();
  }
  return 0;
}
