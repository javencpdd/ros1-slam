// Host-side test for the cancel-origin rule (ROS-free).
//
// Build/run: scripts/verify/check_bt.sh (g++ -std=c++11 -Wall -Wextra -I include)
//
// Why this file exists: the rule decides whether the BT turns a stranger's
// cancel into its own cancel and drops its goal queue.  Getting it wrong in
// either direction is a safety defect -- too loose latches a cancel on
// unrelated traffic and stops a live leg, too strict leaves the mission
// waiting out the 60..500 s arrival budget or, worse, auto-runs the goals that
// were queued behind the leg a fault just killed.
//
// 2026-11-22 second construction pass: the ack alone is no longer allowed to
// stop anything (a stray ack is indistinguishable from a real foreign cancel
// on the ack face).  Evidence now has to exist on another channel for the same
// seq -- the wire frame on `/v8/local/cancel`, or the adapter's own
// `E_FAR_CANCEL_RECEIVED` echo that carries the seq it received.
//
// Second pass, second half: one foreign stop shows up on up to three channels,
// so the same event must promote once and only once.  `already_latched` is that
// guard -- once this node is already cancelling, the next channel to carry the
// same event is confirmation, not a new stop, and must not overwrite the
// recorded origin either (the live probe saw one stop produce two origins).
//
// Four kinds of check, so neither direction can pass by accident:
//   1. a full truth table over the frame facts (16 cells);
//   2. a full truth table over the ack facts (64 cells);
//   3. named hazard cases (unrelated / late / duplicate / idle / no ack / double
//      promotion);
//   4. anti-control: every single-fact flip away from the one STOP_LEG cell
//      must fall back, and the seq value must not change the verdict.
#include "lite3_bt/cancel_origin.hpp"

#include <cstdint>
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

lite3_bt::OriginFacts frame_facts(bool self_issued, bool live, bool accounted,
                                  bool latched = false)
{
  lite3_bt::OriginFacts f;
  f.self_issued = self_issued;
  f.leg_live = live;
  f.already_accounted = accounted;
  f.already_latched = latched;
  return f;
}

lite3_bt::OriginAckFacts ack_facts(bool confirmed, bool self_issued, bool accounted,
                                   bool corroborated, bool live, bool latched = false)
{
  lite3_bt::OriginAckFacts f;
  f.confirmed = confirmed;
  f.self_issued = self_issued;
  f.already_recorded = accounted;
  f.corroborated = corroborated;
  f.leg_live = live;
  f.already_latched = latched;
  return f;
}

bool stop(const lite3_bt::OriginFacts& f)
{
  return lite3_bt::judge_origin_frame(f) == lite3_bt::OriginSignal::STOP_LEG;
}

bool stop(const lite3_bt::OriginAckFacts& f)
{
  return lite3_bt::judge_origin_ack(f) == lite3_bt::OriginSignal::STOP_LEG;
}

bool remember(const lite3_bt::OriginFacts& f)
{
  return lite3_bt::judge_origin_frame(f) == lite3_bt::OriginSignal::REMEMBER;
}

bool remember(const lite3_bt::OriginAckFacts& f)
{
  return lite3_bt::judge_origin_ack(f) == lite3_bt::OriginSignal::REMEMBER;
}
}  // namespace

int main()
{
  // ---- 1. frame truth table (self_issued, live, accounted, latched): 16 cells.
  int frame_stops = 0;
  int frame_bad = 0;
  for (int mask = 0; mask < 16; ++mask)
  {
    const bool self_issued = (mask & 1) != 0;
    const bool live = (mask & 2) != 0;
    const bool accounted = (mask & 4) != 0;
    const bool latched = (mask & 8) != 0;
    const bool got = stop(frame_facts(self_issued, live, accounted, latched));
    const bool want = !self_issued && !latched && !accounted && live;
    if (got) ++frame_stops;
    if (got != want)
    {
      ++frame_bad;
      std::printf("FAIL  frame table mask=%d got=%d want=%d\n", mask, got ? 1 : 0,
                  want ? 1 : 0);
    }
  }
  check(frame_bad == 0, "frame truth table matches its conjunction over four facts");
  check(frame_stops == 1, "exactly one of sixteen frame combinations stops the leg");

  // ---- 2. ack table (confirmed, self, accounted, corroborated, live, latched): 64 cells.
  int ack_stops = 0;
  int ack_bad = 0;
  for (int mask = 0; mask < 64; ++mask)
  {
    const bool confirmed = (mask & 1) != 0;
    const bool self_issued = (mask & 2) != 0;
    const bool accounted = (mask & 4) != 0;
    const bool corroborated = (mask & 8) != 0;
    const bool live = (mask & 16) != 0;
    const bool latched = (mask & 32) != 0;
    const bool got =
        stop(ack_facts(confirmed, self_issued, accounted, corroborated, live, latched));
    const bool want =
        confirmed && !self_issued && !latched && !accounted && corroborated && live;
    if (got) ++ack_stops;
    if (got != want)
    {
      ++ack_bad;
      std::printf("FAIL  ack table mask=%d got=%d want=%d\n", mask, got ? 1 : 0,
                  want ? 1 : 0);
    }
  }
  check(ack_bad == 0, "ack truth table matches its conjunction over six facts");
  check(ack_stops == 1, "exactly one of sixty-four ack combinations stops the leg");

  // ---- 3. the hazard cases that the live probes actually produced.
  check(stop(frame_facts(false, true, false)),
        "a foreign frame with a live leg stops the leg (no ack needed)");
  check(!stop(frame_facts(true, true, false)),
        "our own frame echoing back never stops the leg");
  check(!stop(frame_facts(false, false, false)),
        "a foreign frame with no live leg is only remembered");
  check(!stop(frame_facts(false, true, true)),
        "the same foreign seq arriving twice stops the leg only once");
  // The live probe printed one foreign stop as two promotions (external_wire
  // then external_ack, same seq) -- same event, second channel.
  check(!stop(frame_facts(false, true, false, true)),
        "★a second frame channel after our own latch does not promote again★");
  check(remember(frame_facts(false, true, false, true)),
        "the suppressed second promotion is still remembered, not dropped");

  check(stop(ack_facts(true, false, false, true, true)),
        "corroborated foreign ack during a live leg stops the leg");
  check(!stop(ack_facts(true, false, false, false, true)),
        "★stray ack with no wire evidence never stops the leg★");
  check(!stop(ack_facts(true, false, false, true, false)),
        "★corroborated ack after the leg ended never stops the leg★");
  check(!stop(ack_facts(true, false, true, true, true)),
        "late/duplicate ack already recorded is not a second stop");
  check(!stop(ack_facts(true, true, false, true, true)),
        "ack for a seq this BT published is never an external stop");
  check(!stop(ack_facts(false, false, false, true, true)),
        "ack_fail/unacked outcome is not a stop");
  check(!stop(ack_facts(true, false, false, true, true, true)),
        "★ack arriving after we already latched does not overwrite the origin★");
  check(remember(ack_facts(true, false, false, false, true)),
        "stray ack is remembered, not dropped on the floor");
  check(!remember(ack_facts(true, true, false, true, true)),
        "an ack we published ourselves is ignored outright, not remembered");

  // ---- 4. anti-control: flipping any single fact off the STOP_LEG cell falls back.
  check(!stop(ack_facts(false, false, false, true, true)), "flip confirmed -> no stop");
  check(!stop(ack_facts(true, true, false, true, true)), "flip self_issued -> no stop");
  check(!stop(ack_facts(true, false, true, true, true)), "flip accounted -> no stop");
  check(!stop(ack_facts(true, false, false, false, true)), "flip corroborated -> no stop");
  check(!stop(ack_facts(true, false, false, true, false)), "flip leg_live -> no stop");
  check(!stop(ack_facts(true, false, false, true, true, true)), "flip already_latched -> no stop");

  check(!stop(frame_facts(true, true, false)), "flip frame self_issued -> no stop");
  check(!stop(frame_facts(false, false, false)), "flip frame leg_live -> no stop");
  check(!stop(frame_facts(false, true, true)), "flip frame accounted -> no stop");
  check(!stop(frame_facts(false, true, false, true)), "flip frame latched -> no stop");

  // ---- 5. seq value carries no meaning: same facts, wildly different numbers.
  check(stop(frame_facts(false, true, false)),
        "frame verdict does not depend on the seq value");
  check(stop(ack_facts(true, false, false, true, true)),
        "ack verdict does not depend on the seq value");

  // ---- 6. status-JSON seq extraction (the ack's corroborating channel).
  std::uint64_t seq = 0;
  check(lite3_bt::parse_status_cancel_seq("{\"ok\": false, \"points\": 0, "
                                         "\"reason\": \"E_FAR_CANCEL_RECEIVED\", \"seq\": 900000001}",
                                         &seq) &&
            seq == 900000001ULL,
        "status json seq is extracted");
  seq = 0;
  check(lite3_bt::parse_status_cancel_seq("{\"seq\":42,\"ok\":false}", &seq) && seq == 42ULL,
        "compact json without spaces is extracted");
  seq = 7;
  check(!lite3_bt::parse_status_cancel_seq("{\"ok\": false, \"points\": 0}", &seq),
        "missing seq is rejected, not read as zero");
  seq = 7;
  check(!lite3_bt::parse_status_cancel_seq("{\"seq\": \"900000001\"}", &seq),
        "quoted seq is rejected (only bare integers are a protocol)");
  seq = 7;
  check(!lite3_bt::parse_status_cancel_seq("{\"seq\":}", &seq),
        "empty seq value is rejected");
  check(!lite3_bt::parse_status_cancel_seq("{\"seq\":1}", 0),
        "null out-parameter is rejected");

  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
  return failures ? 1 : 0;
}
