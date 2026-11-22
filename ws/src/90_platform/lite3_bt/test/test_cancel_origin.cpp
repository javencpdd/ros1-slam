// Host-side test for the cancel-ack attribution rule (ROS-free).
//
// Build/run: scripts/verify/check_bt.sh (g++ -std=c++11 -Wall -Wextra -I include)
//
// Why this file exists: the rule decides whether the BT turns a stranger's
// cancel ack into its own cancel.  Getting it wrong in either direction is a
// safety defect -- too loose latches a cancel on unrelated traffic, too strict
// leaves the mission waiting out the 60..500 s arrival budget.  Three kinds of
// check, so neither direction can pass by accident:
//   1. a full truth table over the four facts (no case left unread);
//   2. named cases for the four hazards raised in the 2026-11-22 review
//      (unrelated ack, late ack, duplicate ack, ack after the leg ended);
//   3. anti-control: every single-fact flip away from the one EXTERNAL_STOP
//      combination must fall back to RECORD_ONLY.
#include "lite3_bt/cancel_origin.hpp"

#include <cstdio>

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

lite3_bt::CancelAckFacts facts(bool confirmed, bool self_issued,
                              bool already_recorded, bool has_current,
                              std::uint64_t seq = 900000001)
{
  lite3_bt::CancelAckFacts f;
  f.seq = seq;
  f.confirmed = confirmed;
  f.self_issued = self_issued;
  f.already_recorded = already_recorded;
  f.has_current = has_current;
  return f;
}

bool is_stop(const lite3_bt::CancelAckFacts& f)
{
  return lite3_bt::classify_cancel_ack(f) ==
         lite3_bt::CancelAckDisposition::EXTERNAL_STOP;
}
}  // namespace

int main()
{
  // ---- 1. truth table: EXTERNAL_STOP iff all four facts hold.
  int stops = 0;
  int mismatches = 0;
  for (int mask = 0; mask < 16; ++mask)
  {
    const bool confirmed = (mask & 1) != 0;
    const bool self_issued = (mask & 2) != 0;
    const bool already = (mask & 4) != 0;
    const bool live = (mask & 8) != 0;
    const bool got = is_stop(facts(confirmed, self_issued, already, live));
    const bool want = confirmed && !self_issued && !already && live;
    if (got) ++stops;
    if (got != want)
    {
      ++mismatches;
      std::printf("FAIL  truth table mask=%d got=%d want=%d\n", mask, got ? 1 : 0,
                  want ? 1 : 0);
    }
  }
  check(mismatches == 0, "truth table over the four facts matches the conjunction");
  check(stops == 1, "exactly one of sixteen fact combinations is an external stop");

  // ---- 2. the four hazards from the review.
  check(is_stop(facts(true, false, false, true)),
        "foreign confirmed ack during a live leg is an external stop");
  check(!is_stop(facts(true, false, false, false)),
        "unrelated ack with no live leg is recorded only");
  check(!is_stop(facts(true, false, true, true)),
        "late/duplicate ack already recorded is not a second stop");
  check(!is_stop(facts(true, true, false, true)),
        "ack for a seq this BT published is never an external stop");
  check(!is_stop(facts(false, false, false, true)),
        "ack_fail/unacked outcome is not a stop");
  check(!is_stop(facts(true, true, true, false)),
        "own late ack on an idle tree stays inert");

  // ---- 3. anti-control: flipping any single fact must fall back.
  check(!is_stop(facts(false, false, false, true)), "flip confirmed -> recorded");
  check(!is_stop(facts(true, true, false, true)), "flip self_issued -> recorded");
  check(!is_stop(facts(true, false, true, true)), "flip already_recorded -> recorded");
  check(!is_stop(facts(true, false, false, false)), "flip has_current -> recorded");

  // ---- 4. the seq value carries no meaning: same facts, wildly different seq.
  check(is_stop(facts(true, false, false, true, 1)),
        "small seq with the right facts is still an external stop");
  check(is_stop(facts(true, false, false, true, 900000007)),
        "adapter-band seq with the right facts is still an external stop");
  check(is_stop(facts(true, false, false, true, 0)),
        "unknown/zero seq with the right facts is still an external stop");

  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
  return failures ? 1 : 0;
}
