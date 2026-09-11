// Copyright 2026 selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef AUTOWARE__PATH_OPTIMIZER__UTILS__DRIVABLE_AREA_RECOVERY_HPP_
#define AUTOWARE__PATH_OPTIMIZER__UTILS__DRIVABLE_AREA_RECOVERY_HPP_

#include <cmath>

namespace autoware::path_optimizer
{
// One cold optimization per continuous boundary-stop episode. A successful cycle alone must
// not cause repeated resets if the boundary decision flickers. All times are ROS time.
class DrivableAreaRecoveryState
{
public:
  bool update(double now, bool outside, bool eligible)
  {
    if (!std::isfinite(now)) {
      reset();
      return false;
    }
    if (have_last_ && (now < last_ || now - last_ > 1.0)) reset();
    last_ = now;
    have_last_ = true;
    if (!outside) {
      if (!have_clear_since_) clear_since_ = now;
      have_clear_since_ = true;
      if (now - clear_since_ >= 1.0) retried_ = false;
      return false;
    }
    have_clear_since_ = false;
    if (!eligible || retried_) return false;
    retried_ = true;
    return true;
  }

  void reset()
  {
    have_last_ = false;
    have_clear_since_ = false;
    retried_ = false;
  }

private:
  double last_{0.0};
  double clear_since_{0.0};
  bool have_last_{false};
  bool have_clear_since_{false};
  bool retried_{false};
};
}  // namespace autoware::path_optimizer
#endif
