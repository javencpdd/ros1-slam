// Pure wire-format and decision helpers for BT cancel / recovery / goal policy.
//
// ROS-free so the same rules can be exercised on a plain host with g++.
// Wire formats (deliberately flat, no JSON dependency):
//   cancel      "<seq>:<scope>:<reason>"   scope = current | all
//   cancel ack  "<seq>:<outcome>"          outcome = cancelled | ack_fail | unacked
//   goal policy "dwell=1.5;arrival=90;path=30;retry=2;index=1;total=3;label=up"
#ifndef LITE3_BT_BT_POLICY_HPP
#define LITE3_BT_BT_POLICY_HPP

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>

namespace lite3_bt
{

struct GoalPolicy
{
  double dwell_s = 0.0;          // 0 = tree/launch default
  double arrival_timeout_s = 0.0;
  double path_timeout_s = 0.0;
  int retry_max = -1;            // -1 = launch default
  int goal_index = 0;            // 0 = derive in the BT
  int goals_total = 0;
  std::string label;
};

enum class RecoveryAction { RETRY, SKIP, ABORT };
struct RecoveryDecision
{
  RecoveryAction action;
  bool exhausted;
};

inline bool parse_uint(const std::string& text, std::uint64_t* out)
{
  if (out == 0 || text.empty()) return false;
  for (std::size_t i = 0; i < text.size(); ++i)
    if (text[i] < '0' || text[i] > '9') return false;
  *out = static_cast<std::uint64_t>(std::strtoull(text.c_str(), 0, 10));
  return true;
}

inline bool parse_double(const std::string& text, double* out)
{
  if (out == 0 || text.empty()) return false;
  char* end = 0;
  const double value = std::strtod(text.c_str(), &end);
  if (end == 0 || *end != '\0' || !std::isfinite(value)) return false;
  *out = value;
  return true;
}

inline bool parse_nonnegative(const std::string& text, double* out)
{
  return parse_double(text, out) && *out >= 0.0;
}

inline bool parse_int(const std::string& text, int* out)
{
  if (out == 0 || text.empty()) return false;
  char* end = 0;
  const long value = std::strtol(text.c_str(), &end, 10);
  if (end == 0 || *end != '\0' || value < 0 || value > 1000000) return false;
  *out = static_cast<int>(value);
  return true;
}

// "<seq>:<scope>:<reason>"; reason may itself contain ':' but not ';'.
inline bool parse_cancel(const std::string& text, std::uint64_t* seq, std::string* scope,
                         std::string* reason)
{
  if (seq == 0 || scope == 0 || reason == 0) return false;
  const std::size_t first = text.find(':');
  if (first == std::string::npos) return false;
  if (!parse_uint(text.substr(0, first), seq)) return false;
  const std::size_t second = text.find(':', first + 1);
  const std::string head = second == std::string::npos
      ? text.substr(first + 1) : text.substr(first + 1, second - first - 1);
  if (head != "current" && head != "all") return false;
  *scope = head;
  *reason = second == std::string::npos ? std::string() : text.substr(second + 1);
  if (reason->size() > 200 || reason->find(';') != std::string::npos
      || reason->find('\n') != std::string::npos)
    return false;
  return true;
}

// "<seq>:<outcome>"; only the three known outcomes advance cancel state.
inline bool parse_cancel_ack(const std::string& text, std::uint64_t* seq, std::string* outcome)
{
  if (seq == 0 || outcome == 0) return false;
  const std::size_t colon = text.find(':');
  if (colon == std::string::npos) return false;
  if (!parse_uint(text.substr(0, colon), seq)) return false;
  const std::string tail = text.substr(colon + 1);
  if (tail != "cancelled" && tail != "ack_fail" && tail != "unacked") return false;
  *outcome = tail;
  return true;
}

inline bool cancel_ack_advances(std::uint64_t ack_seq, const std::string& outcome,
                                std::uint64_t current_seq)
{
  return ack_seq == current_seq && outcome != "unacked";
}

// "key=value;key=value"; unknown keys are refused so a typo cannot be silent.
inline bool parse_goal_policy(const std::string& text, GoalPolicy* out)
{
  if (out == 0) return false;
  GoalPolicy value;
  if (!text.empty())
  {
    std::size_t start = 0;
    while (start <= text.size())
    {
      const std::size_t end = text.find(';', start);
      const std::string pair = text.substr(start, end == std::string::npos ? std::string::npos
                                                                          : end - start);
      if (!pair.empty())
      {
        const std::size_t eq = pair.find('=');
        if (eq == std::string::npos) return false;
        const std::string key = pair.substr(0, eq);
        const std::string raw = pair.substr(eq + 1);
        if (key == "dwell")
        {
          if (!parse_nonnegative(raw, &value.dwell_s)) return false;
        }
        else if (key == "arrival")
        {
          if (!parse_nonnegative(raw, &value.arrival_timeout_s)) return false;
        }
        else if (key == "path")
        {
          if (!parse_nonnegative(raw, &value.path_timeout_s)) return false;
        }
        else if (key == "retry")
        {
          if (!parse_int(raw, &value.retry_max)) return false;
        }
        else if (key == "index")
        {
          if (!parse_int(raw, &value.goal_index)) return false;
        }
        else if (key == "total")
        {
          if (!parse_int(raw, &value.goals_total)) return false;
        }
        else if (key == "label")
        {
          if (raw.size() > 80 || raw.find('\n') != std::string::npos) return false;
          value.label = raw;
        }
        else
        {
          return false;
        }
      }
      if (end == std::string::npos) break;
      start = end + 1;
    }
  }
  *out = value;
  return true;
}

// Mission goal: pose + policy in one message, so goal order cannot race a
// separate policy topic.  Format:
//   "x=1;y=2;z=0;yaw=0;dwell=1.5;arrival=90;path=30;retry=2;index=1;total=3;label=up"
inline bool parse_mission_goal(const std::string& text, double* x, double* y, double* z,
                               double* yaw, GoalPolicy* policy)
{
  if (x == 0 || y == 0 || z == 0 || yaw == 0 || policy == 0) return false;
  bool has_x = false, has_y = false, has_z = false;
  double px = 0.0, py = 0.0, pz = 0.0, pyaw = 0.0;
  GoalPolicy value;
  std::size_t start = 0;
  while (start <= text.size())
  {
    const std::size_t end = text.find(';', start);
    const std::string pair = text.substr(start, end == std::string::npos ? std::string::npos
                                                                        : end - start);
    if (!pair.empty())
    {
      const std::size_t eq = pair.find('=');
      if (eq == std::string::npos) return false;
      const std::string key = pair.substr(0, eq);
      const std::string raw = pair.substr(eq + 1);
      if (key == "x") { if (!parse_double(raw, &px)) return false; has_x = true; }
      else if (key == "y") { if (!parse_double(raw, &py)) return false; has_y = true; }
      else if (key == "z") { if (!parse_double(raw, &pz)) return false; has_z = true; }
      else if (key == "yaw") { if (!parse_double(raw, &pyaw)) return false; }
      else if (key == "dwell") { if (!parse_nonnegative(raw, &value.dwell_s)) return false; }
      else if (key == "arrival") { if (!parse_nonnegative(raw, &value.arrival_timeout_s)) return false; }
      else if (key == "path") { if (!parse_nonnegative(raw, &value.path_timeout_s)) return false; }
      else if (key == "retry") { if (!parse_int(raw, &value.retry_max)) return false; }
      else if (key == "index") { if (!parse_int(raw, &value.goal_index)) return false; }
      else if (key == "total") { if (!parse_int(raw, &value.goals_total)) return false; }
      else if (key == "label")
      {
        if (raw.size() > 80 || raw.find('\n') != std::string::npos) return false;
        value.label = raw;
      }
      else return false;
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
  if (!has_x || !has_y || !has_z) return false;
  *x = px; *y = py; *z = pz; *yaw = pyaw; *policy = value;
  return true;
}

// retry/replan retry up to retry_max; skip/abort act immediately.  When a
// retrying policy is exhausted, on_exhausted picks abort or skip.
inline RecoveryDecision decide_recovery(const std::string& mode, int attempts, int retry_max,
                                        const std::string& on_exhausted)
{
  if (mode == "retry" || mode == "replan")
  {
    if (attempts < retry_max) return {RecoveryAction::RETRY, false};
    return {on_exhausted == "abort" ? RecoveryAction::ABORT : RecoveryAction::SKIP, true};
  }
  if (mode == "abort") return {RecoveryAction::ABORT, false};
  return {RecoveryAction::SKIP, false};
}

}  // namespace lite3_bt

#endif  // LITE3_BT_BT_POLICY_HPP
