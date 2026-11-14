// Host-only tests for the cancel/recovery/goal-policy wire helpers.
#include <lite3_bt/bt_policy.hpp>

#include <cassert>
#include <cstdio>
#include <string>

using namespace lite3_bt;

int main()
{
  std::uint64_t seq = 0;
  std::string scope, reason, outcome;

  assert(parse_cancel("7:all:operator", &seq, &scope, &reason));
  assert(seq == 7 && scope == "all" && reason == "operator");
  assert(parse_cancel("8:current", &seq, &scope, &reason));
  assert(seq == 8 && scope == "current" && reason.empty());
  assert(!parse_cancel("7:bad:x", &seq, &scope, &reason));
  assert(!parse_cancel("x:all:", &seq, &scope, &reason));
  assert(!parse_cancel("7:all:bad;reason", &seq, &scope, &reason));

  assert(parse_cancel_ack("7:cancelled", &seq, &outcome));
  assert(seq == 7 && outcome == "cancelled");
  assert(!parse_cancel_ack("7:ok", &seq, &outcome));
  assert(!parse_cancel_ack("7:cancelled:extra", &seq, &outcome));
  assert(cancel_ack_advances(7, "cancelled", 7));
  assert(!cancel_ack_advances(6, "cancelled", 7));
  assert(!cancel_ack_advances(7, "unacked", 7));

  GoalPolicy policy;
  assert(parse_goal_policy(
      "dwell=1.5;arrival=90;path=30;retry=2;index=1;total=3;label=up", &policy));
  assert(policy.dwell_s == 1.5 && policy.arrival_timeout_s == 90.0);
  assert(policy.path_timeout_s == 30.0 && policy.retry_max == 2);
  assert(policy.goal_index == 1 && policy.goals_total == 3 && policy.label == "up");
  assert(parse_goal_policy("", &policy));
  assert(policy.retry_max == -1 && policy.dwell_s == 0.0);
  assert(!parse_goal_policy("dwell=-1", &policy));
  double mx = 0.0, my = 0.0, mz = 0.0, myaw = 0.0;
  assert(parse_mission_goal("x=-5.9;y=1.1;z=-0.2;yaw=-1.57;index=1;total=2;path=1.0",
                            &mx, &my, &mz, &myaw, &policy));
  assert(mx == -5.9 && my == 1.1 && mz == -0.2 && myaw == -1.57);
  assert(policy.path_timeout_s == 1.0 && policy.goals_total == 2);
  assert(!parse_goal_policy("unknown=1", &policy));
  assert(!parse_goal_policy("dwell", &policy));

  RecoveryDecision decision = decide_recovery("retry", 0, 2, "skip");
  assert(decision.action == RecoveryAction::RETRY && !decision.exhausted);
  decision = decide_recovery("retry", 2, 2, "skip");
  assert(decision.action == RecoveryAction::SKIP && decision.exhausted);
  decision = decide_recovery("replan", 1, 2, "abort");
  assert(decision.action == RecoveryAction::RETRY && !decision.exhausted);
  decision = decide_recovery("replan", 2, 2, "abort");
  assert(decision.action == RecoveryAction::ABORT && decision.exhausted);
  decision = decide_recovery("abort", 0, 2, "skip");
  assert(decision.action == RecoveryAction::ABORT && !decision.exhausted);
  decision = decide_recovery("skip", 0, 2, "skip");
  assert(decision.action == RecoveryAction::SKIP && !decision.exhausted);

  std::printf("BT POLICY OK\n");
  return 0;
}
