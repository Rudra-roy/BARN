// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_dynamic_tracking/kalman.hpp"

namespace barn_dynamic_tracking
{

void ConstantVelocityKF1D::init(double pos, double p_pos, double p_vel)
{
  x_[0] = pos;
  x_[1] = 0.0;
  p_[0][0] = p_pos;
  p_[0][1] = 0.0;
  p_[1][0] = 0.0;
  p_[1][1] = p_vel;
  initialised_ = true;
}

void ConstantVelocityKF1D::predict(double dt, double q)
{
  // State transition F = [[1, dt], [0, 1]].
  x_[0] += dt * x_[1];

  // P = F P F^T + Q, with Q the discrete white-noise-acceleration model.
  const double p00 = p_[0][0];
  const double p01 = p_[0][1];
  const double p10 = p_[1][0];
  const double p11 = p_[1][1];

  // F P F^T
  const double fp00 = p00 + dt * p10 + dt * (p01 + dt * p11);
  const double fp01 = p01 + dt * p11;
  const double fp10 = p10 + dt * p11;
  const double fp11 = p11;

  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;

  p_[0][0] = fp00 + q * dt4 / 4.0;
  p_[0][1] = fp01 + q * dt3 / 2.0;
  p_[1][0] = fp10 + q * dt3 / 2.0;
  p_[1][1] = fp11 + q * dt2;
}

void ConstantVelocityKF1D::update(double measurement, double r)
{
  // Measurement H = [1, 0]; innovation covariance S = P00 + r.
  const double s = p_[0][0] + r;
  const double k0 = p_[0][0] / s;  // Kalman gain
  const double k1 = p_[1][0] / s;

  const double y = measurement - x_[0];
  x_[0] += k0 * y;
  x_[1] += k1 * y;

  // P = (I - K H) P
  const double p00 = p_[0][0];
  const double p01 = p_[0][1];
  const double p10 = p_[1][0];
  const double p11 = p_[1][1];
  p_[0][0] = (1.0 - k0) * p00;
  p_[0][1] = (1.0 - k0) * p01;
  p_[1][0] = p10 - k1 * p00;
  p_[1][1] = p11 - k1 * p01;
}

}  // namespace barn_dynamic_tracking
