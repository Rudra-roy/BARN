// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_CLASSICAL__CONTROLLER_HPP_
#define BARN_CLASSICAL__CONTROLLER_HPP_

#include <string>
#include <vector>

#include "barn_classical/local_planner.hpp"
#include "barn_core/distance_field.hpp"
#include "barn_core/types.hpp"

namespace barn_classical
{

struct MpcParams
{
  int horizon{20};
  double dt{0.1};
  double max_speed{2.0};
  double max_yaw_rate{1.5};
  double max_accel{2.5};
  double max_yaw_accel{3.0};
  double solve_deadline_ms{35.0};
  int max_linearization_passes{3};
  double obstacle_margin{0.02};
  double max_obstacle_slack{0.40};
  // Extra keep-out clearance to moving obstacles, on top of the summed radii
  // (obstacle radius + robot circumscribed radius). [m]
  double dynamic_margin{0.20};
  // Max allowed soft penetration of the dynamic keep-out [m]. Kept well below
  // the keep-out radius (unlike the roomy static slack) so avoidance stays firm.
  double max_dynamic_slack{0.35};
  // Cap on how many tracked obstacles contribute constraints (nearest first).
  int max_dynamic_obstacles{8};
  Footprint footprint{};
};

/// A tracked moving obstacle in the MPC planning frame. Its future position is
/// predicted with a constant-velocity model over the horizon.
struct DynamicObstacle
{
  double x{0.0};       ///< current centroid x [m]
  double y{0.0};       ///< current centroid y [m]
  double vx{0.0};      ///< velocity x [m/s]
  double vy{0.0};      ///< velocity y [m/s]
  double radius{0.0};  ///< obstacle radius [m]
};

struct MpcResult
{
  barn_core::VelocityCommand command{};
  Path2D prediction;
  bool success{false};
  bool timed_out{false};
  double solve_ms{0.0};
  std::string status{"not_run"};
};

/// Sequentially-linearized differential-drive MPC backed by OSQP.
class Controller
{
public:
  explicit Controller(const MpcParams & params = {}) : params_(params) {}

  MpcResult control(
    const LocalTrajectory & trajectory, const barn_core::State2D & state,
    const barn_core::DistanceField2D & distance_field,
    const std::vector<DynamicObstacle> & obstacles = {});

  void reset();

private:
  MpcParams params_;
  std::vector<double> warm_start_;
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__CONTROLLER_HPP_
