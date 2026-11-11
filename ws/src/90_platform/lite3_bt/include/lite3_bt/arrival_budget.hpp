#ifndef LITE3_BT_ARRIVAL_BUDGET_HPP
#define LITE3_BT_ARRIVAL_BUDGET_HPP

#include <algorithm>

namespace lite3_bt
{
// Wall-clock budget for WaitArrival.  The tree's timeout stays the floor, so
// short goals keep the old behaviour; longer paths get budget proportional to
// their length.  Without this a single tree value has to fit both a 2 m flat
// leg (well under 60 s) and the 10.0 m ramp polyline that took 141 s wall at
// RTF 0.45 on 2026-11-12.  `speed_floor_mps` is the wall-clock progress below
// which the run is treated as stalled, not slow (default 0.02 => that ramp
// gets 502 s, ~3.5x the measured traversal, still covering a drop to the
// project RTF floor of 0.25).
inline double arrival_budget_s(double base_s, double path_length_m,
                               double speed_floor_mps)
{
  if (!(speed_floor_mps > 0.0) || !(path_length_m > 0.0)) return base_s;
  return std::max(base_s, path_length_m / speed_floor_mps);
}
}  // namespace lite3_bt

#endif  // LITE3_BT_ARRIVAL_BUDGET_HPP
