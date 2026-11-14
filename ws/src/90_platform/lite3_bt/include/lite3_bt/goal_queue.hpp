// Header-only, ROS-free helpers for the BT console demo goal queue.
//
// Kept free of ROS/BT.CPP types so the queue and the planner-ack protocol can
// be compiled and exercised on a plain host with g++, without a ROS master.
// The BT node remains a goal OWNER: it never computes a path.  It only decides
// which queued goal to publish next, and (optionally) waits for the downstream
// planner to report that goal's outcome.
#ifndef LITE3_BT_GOAL_QUEUE_HPP
#define LITE3_BT_GOAL_QUEUE_HPP

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <string>

namespace lite3_bt
{

template <typename T>
class GoalQueue
{
public:
  explicit GoalQueue(std::size_t max_size = 16) : max_size_(max_size ? max_size : 1) {}

  void set_max(std::size_t max_size) { max_size_ = max_size ? max_size : 1; }
  std::size_t max_size() const { return max_size_; }
  std::size_t depth() const { return items_.size(); }
  bool empty() const { return items_.empty(); }

  // Refuse instead of evicting an older goal.  A silent overwrite is exactly
  // the multi-goal loss this queue exists to prevent.
  bool push(const T& item)
  {
    if (items_.size() >= max_size_) return false;
    items_.push_back(item);
    return true;
  }

  bool pop(T& out)
  {
    if (items_.empty()) return false;
    out = items_.front();
    items_.pop_front();
    return true;
  }

  // Retry/replan puts the failed goal back at the head so the next
  // WaitForGoal tick reuses it instead of losing it behind newer goals.
  bool push_front(const T& item)
  {
    if (items_.size() >= max_size_) return false;
    items_.push_front(item);
    return true;
  }

  void clear() { items_.clear(); }

private:
  std::size_t max_size_;
  std::deque<T> items_;
};

// Planner result wire format: "<seq>:ok" or "<seq>:fail".
// Anything else (missing colon, non-numeric seq, unknown outcome, empty) is
// refused so a malformed message can never be read as a success.
inline bool parse_planner_ack(const std::string& text, std::uint64_t* seq, bool* ok)
{
  if (seq == 0 || ok == 0) return false;
  const std::size_t colon = text.find(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 >= text.size()) return false;
  const std::string head = text.substr(0, colon);
  for (std::size_t i = 0; i < head.size(); ++i)
    if (head[i] < '0' || head[i] > '9') return false;
  const std::string tail = text.substr(colon + 1);
  if (tail == "ok") *ok = true;
  else if (tail == "fail") *ok = false;
  else return false;
  *seq = static_cast<std::uint64_t>(std::strtoull(head.c_str(), 0, 10));
  return true;
}

// Only an OK ack for the CURRENT goal advances the queue.  A stale ack from a
// previous goal, or a FAIL ack, never advances it.
inline bool ack_advances(std::uint64_t ack_seq, bool ack_ok, std::uint64_t current_seq)
{
  return ack_ok && ack_seq == current_seq;
}

// /global_path/status carries a JSON object built by Python json.dumps, so
// both ": " and ":" must be accepted.  Only a small, explicit subset is read:
// ok / points / reason.  A malformed status is refused rather than guessed.
inline bool parse_path_status(const std::string& text, bool* ok, int* points,
                              std::string* reason)
{
  if (ok == 0 || points == 0) return false;
  const std::string ok_key = "\"ok\":";
  const std::size_t ok_pos = text.find(ok_key);
  if (ok_pos == std::string::npos) return false;
  std::size_t i = ok_pos + ok_key.size();
  while (i < text.size() && (text[i] == ' ' || text[i] == '\t')) ++i;
  if (text.compare(i, 4, "true") == 0) *ok = true;
  else if (text.compare(i, 5, "false") == 0) *ok = false;
  else return false;

  *points = 0;
  const std::string points_key = "\"points\":";
  const std::size_t p = text.find(points_key);
  if (p != std::string::npos)
  {
    std::size_t b = p + points_key.size();
    while (b < text.size() && (text[b] == ' ' || text[b] == '\t')) ++b;
    std::size_t e = b;
    while (e < text.size() && text[e] >= '0' && text[e] <= '9') ++e;
    if (e > b) *points = static_cast<int>(std::strtol(text.substr(b, e - b).c_str(), 0, 10));
    else if (*ok) return false;
  }

  if (reason != 0)
  {
    reason->clear();
    const std::string reason_key = "\"reason\":";
    const std::size_t r = text.find(reason_key);
    if (r != std::string::npos)
    {
      const std::size_t q1 = text.find('"', r + reason_key.size());
      if (q1 != std::string::npos)
      {
        const std::size_t q2 = text.find('"', q1 + 1);
        if (q2 != std::string::npos) *reason = text.substr(q1 + 1, q2 - q1 - 1);
      }
    }
  }
  return true;
}

}  // namespace lite3_bt

#endif  // LITE3_BT_GOAL_QUEUE_HPP
