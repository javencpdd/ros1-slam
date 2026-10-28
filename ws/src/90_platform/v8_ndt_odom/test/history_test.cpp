#include <v8_ndt_odom/pose_history.h>
#include <iostream>
#include <limits>

static bool near(float a, float b) { return std::fabs(a - b) < 1e-5f; }

int main() {
  v8_ndt_odom::PoseHistory history;
  Eigen::Matrix4f first = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f second = first;
  second(0, 3) = 0.10f;
  history.reset(first);
  if (!history.observe(first, 1.0, true)
      || !history.observe(second, 2.0, true)
      || !near(history.predict(true)(0, 3), 0.20f)
      || !near(history.predict(false)(0, 3), 0.10f)) return 1;

  // A rejected alignment may not manufacture a new odometry receipt timestamp.
  Eigen::Matrix4f rejected = first;
  rejected(0, 3) = 5.0f;
  if (history.observe(rejected, 3.0, false) || history.stamp() != 2.0
      || history.count() != 2 || !near(history.current()(0, 3), 0.10f)
      || !near(history.predict(true)(0, 3), 0.20f)) return 2;
  if (history.observe(rejected, 1.5, true) || history.stamp() != 2.0) return 3;

  history.reset(first);
  if (history.count() || history.stamp() != 0.0
      || !near(history.predict(true)(0, 3), 0.0f)) return 4;
  first(0, 0) = std::numeric_limits<float>::quiet_NaN();
  if (history.observe(first, 4.0, true)) return 5;
  std::cout << "{\"two_frame_prediction\":true,\"rejected_frame_not_fresh\":true,"
               "\"stale_stamp_rejected\":true,\"reset_clears_history\":true,"
               "\"nonfinite_rejected\":true}" << std::endl;
  return 0;
}
