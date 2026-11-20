// Progress patience for WaitArrival (MBF controller "patience"/PAT_EXCEEDED
// analogue).  ROS-free so it can be exercised on a plain host with g++.
//
// Semantics: while driving to a goal, the remaining distance must improve by
// at least `epsilon_m` at least once every `patience_s` seconds; otherwise the
// leg is reported stalled.  patience_s <= 0 disables the check (default), so
// existing trees keep their previous behavior.
#ifndef LITE3_BT_ARRIVAL_PROGRESS_HPP
#define LITE3_BT_ARRIVAL_PROGRESS_HPP

namespace lite3_bt
{

struct ProgressTracker
{
  double patience_s = 0.0;      // <= 0 disables the check
  double epsilon_m = 0.05;      // improvement that counts as progress
  double best_remaining_m = -1.0;
  double last_progress_s = 0.0;

  void reset(double now_s)
  {
    best_remaining_m = -1.0;
    last_progress_s = now_s;
  }

  // Call once per tick.  Returns true when no progress was made for longer
  // than patience_s.
  bool stalled(double remaining_m, double now_s)
  {
    if (patience_s <= 0.0) return false;
    if (best_remaining_m < 0.0 || remaining_m < best_remaining_m - epsilon_m)
    {
      best_remaining_m = remaining_m;
      last_progress_s = now_s;
      return false;
    }
    return (now_s - last_progress_s) > patience_s;
  }
};

}  // namespace lite3_bt

#endif  // LITE3_BT_ARRIVAL_PROGRESS_HPP
