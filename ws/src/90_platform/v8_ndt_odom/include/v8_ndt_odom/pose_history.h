#pragma once

#include <Eigen/Geometry>
#include <cmath>

namespace v8_ndt_odom {

// Only accepted observations advance history or freshness.  Keeping the previous
// transform before committing current fixes the former identity-motion predictor.
class PoseHistory {
 public:
  void reset(const Eigen::Matrix4f& seed) {
    current_ = previous_ = seed;
    count_ = 0;
    stamp_ = 0.0;
  }

  bool observe(const Eigen::Matrix4f& candidate, double stamp, bool accepted) {
    if (!accepted || !candidate.allFinite() || !std::isfinite(stamp)
        || stamp <= 0.0 || (count_ && stamp <= stamp_)) return false;
    previous_ = current_;
    current_ = candidate;
    stamp_ = stamp;
    ++count_;
    return true;
  }

  Eigen::Matrix4f predict(bool enabled) const {
    if (!enabled || count_ < 2) return current_;
    const Eigen::Matrix4f delta = previous_.inverse() * current_;
    const float yaw = std::atan2(delta(1, 0), delta(0, 0));
    Eigen::Matrix4f step = Eigen::Matrix4f::Identity();
    step(0, 0) = step(1, 1) = std::cos(yaw);
    step(1, 0) = std::sin(yaw);
    step(0, 1) = -std::sin(yaw);
    // Preserve the existing turn guard; this repair changes history ownership only.
    if (std::fabs(yaw) <= 0.05f) {
      step(0, 3) = delta(0, 3);
      step(1, 3) = delta(1, 3);
    }
    return current_ * step;
  }

  double stamp() const { return stamp_; }
  unsigned count() const { return count_; }
  const Eigen::Matrix4f& current() const { return current_; }

 private:
  Eigen::Matrix4f previous_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f current_ = Eigen::Matrix4f::Identity();
  unsigned count_ = 0;
  double stamp_ = 0.0;
};

}  // namespace v8_ndt_odom
