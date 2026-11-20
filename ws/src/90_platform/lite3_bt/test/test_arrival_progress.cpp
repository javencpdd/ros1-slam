// Pure-function checks for the WaitArrival progress patience
// (MBF PAT_EXCEEDED analogue).  No ROS, plain host g++.
#include <lite3_bt/arrival_progress.hpp>

#include <cstdio>

static int failures = 0;

static void check(bool cond, const char* what)
{
  std::printf("%s  %s\n", cond ? "PASS" : "FAIL", what);
  if (!cond) ++failures;
}

int main()
{
  lite3_bt::ProgressTracker t;
  t.patience_s = 5.0;
  t.epsilon_m = 0.05;
  t.reset(0.0);
  check(!t.stalled(3.0, 0.1), "first sample arms the tracker");
  check(!t.stalled(2.9, 4.0), "improvement keeps it running");
  check(t.stalled(2.9, 9.5), "flat distance past patience stalls");
  check(!t.stalled(2.7, 10.0), ">= epsilon improvement resets the clock");
  check(t.stalled(2.7, 15.5), "stall restarts after the reset");
  check(t.stalled(2.66, 16.0), "sub-epsilon improvement does not reset (still stalled)");
  check(t.stalled(2.66, 21.5), "sub-epsilon improvement still stalls");

  t.reset(100.0);
  check(!t.stalled(1.0, 100.0), "reset re-arms with the new sample");
  check(t.stalled(1.0, 105.5), "reset keeps honesty (stall after patience)");

  lite3_bt::ProgressTracker off;
  off.patience_s = 0.0;
  off.reset(0.0);
  check(!off.stalled(10.0, 0.0), "patience 0 disables the check");
  check(!off.stalled(10.0, 1e6), "patience 0 never stalls");

  lite3_bt::ProgressTracker neg;
  neg.patience_s = -1.0;
  neg.reset(0.0);
  check(!neg.stalled(10.0, 1e6), "negative patience disables the check");
  return failures ? 1 : 0;
}
