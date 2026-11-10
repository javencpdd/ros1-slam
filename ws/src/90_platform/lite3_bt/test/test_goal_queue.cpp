// Host-side test for the ROS-free goal queue and planner-ack protocol.
// Build: g++ -std=c++11 -Wall -Wextra -I../../include test_goal_queue.cpp -o /tmp/...
#include "lite3_bt/goal_queue.hpp"

#include <cstdio>
#include <string>

namespace
{
int failures = 0;

void check(bool cond, const char* what)
{
  if (!cond)
  {
    ++failures;
    std::printf("FAIL  %s\n", what);
  }
  else
  {
    std::printf("PASS  %s\n", what);
  }
}
}  // namespace

int main()
{
  lite3_bt::GoalQueue<int> q(2);
  int out = -1;
  check(!q.pop(out), "empty queue refuses pop");
  check(q.push(7) && q.push(8), "two goals accepted");
  check(!q.push(9), "third goal refused at capacity");
  check(q.depth() == 2, "capacity overflow does not grow queue");
  check(q.pop(out) && out == 7, "first out is first in");
  check(q.pop(out) && out == 8, "second out is second in");
  check(!q.pop(out), "queue empty after drain");
  check(q.push(11) && q.push(12), "queue reusable after draining");
  q.clear();
  check(q.empty(), "clear empties the queue");

  q.set_max(1);
  check(q.max_size() == 1, "set_max applies");
  check(q.push(21) && !q.push(22), "max_size=1 refuses the extra goal");
  q.set_max(0);
  check(q.max_size() == 1, "zero max is clamped, not an unbounded queue");

  std::uint64_t seq = 0;
  bool ok = false;
  check(lite3_bt::parse_planner_ack("3:ok", &seq, &ok) && seq == 3 && ok,
        "ack 3:ok parses");
  check(lite3_bt::parse_planner_ack("12:fail", &seq, &ok) && seq == 12 && !ok,
        "ack 12:fail parses");
  check(!lite3_bt::parse_planner_ack("", &seq, &ok), "empty ack refused");
  check(!lite3_bt::parse_planner_ack("3", &seq, &ok), "missing colon refused");
  check(!lite3_bt::parse_planner_ack(":ok", &seq, &ok), "missing seq refused");
  check(!lite3_bt::parse_planner_ack("x:ok", &seq, &ok), "non-numeric seq refused");
  check(!lite3_bt::parse_planner_ack("3:yes", &seq, &ok), "unknown outcome refused");
  check(!lite3_bt::parse_planner_ack("3:", &seq, &ok), "empty outcome refused");
  check(!lite3_bt::parse_planner_ack("3:ok", 0, &ok), "null seq pointer refused");
  check(!lite3_bt::parse_planner_ack("3:ok", &seq, 0), "null ok pointer refused");

  check(lite3_bt::ack_advances(3, true, 3), "matching ok advances");
  check(!lite3_bt::ack_advances(2, true, 3), "stale ack does not advance");
  check(!lite3_bt::ack_advances(4, true, 3), "future ack does not advance");
  check(!lite3_bt::ack_advances(3, false, 3), "fail ack does not advance");

  bool st_ok = false;
  int st_points = -1;
  std::string st_reason;
  check(lite3_bt::parse_path_status("{\"ok\": true, \"points\": 12}", &st_ok,
                                   &st_points, &st_reason) &&
        st_ok && st_points == 12, "path status ok with 12 points parses");
  check(lite3_bt::parse_path_status("{\"ok\":false,\"reason\":\"E_PLAN_RUNNING\"}",
                                    &st_ok, &st_points, &st_reason) &&
        !st_ok && st_points == 0 && st_reason == "E_PLAN_RUNNING",
        "path status busy reason parses");
  check(!lite3_bt::parse_path_status("{}", &st_ok, &st_points, &st_reason),
        "status without ok refused");
  check(!lite3_bt::parse_path_status("{\"ok\": maybe}", &st_ok, &st_points, &st_reason),
        "status with unknown ok value refused");
  check(!lite3_bt::parse_path_status("{\"ok\": true, \"points\": }", &st_ok,
                                    &st_points, &st_reason),
        "successful status without a point count refused");
  check(!lite3_bt::parse_path_status("{\"ok\": true, \"points\": 3}", 0,
                                    &st_points, &st_reason),
        "null ok pointer refused");

  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
  return failures ? 1 : 0;
}
