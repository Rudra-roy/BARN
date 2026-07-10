// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// A minimal, dependency-free scalar constant-velocity Kalman filter. Two of
// these (x and y) track one obstacle centroid. This is REAL and unit-tested;
// the surrounding clustering/association/TTC logic is currently stubbed.

#ifndef BARN_DYNAMIC_TRACKING__KALMAN_HPP_
#define BARN_DYNAMIC_TRACKING__KALMAN_HPP_

namespace barn_dynamic_tracking
{

/// State [position, velocity] with a scalar position measurement.
class ConstantVelocityKF1D
{
public:
  ConstantVelocityKF1D() = default;

  /// Initialise at a measured position with zero velocity and large covariance.
  void init(double pos, double p_pos = 1.0, double p_vel = 10.0);

  /// Predict forward by `dt` seconds. `q` is the acceleration process-noise
  /// spectral density (m^2/s^3).
  void predict(double dt, double q = 1.0);

  /// Fuse a position measurement with variance `r` (m^2).
  void update(double measurement, double r = 0.05);

  double position() const { return x_[0]; }
  double velocity() const { return x_[1]; }
  bool initialised() const { return initialised_; }

private:
  double x_[2]{0.0, 0.0};        // [pos, vel]
  double p_[2][2]{{1.0, 0.0}, {0.0, 10.0}};  // covariance
  bool initialised_{false};
};

}  // namespace barn_dynamic_tracking

#endif  // BARN_DYNAMIC_TRACKING__KALMAN_HPP_
