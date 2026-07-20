// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// DynamicObstacle: a Gazebo Sim (Harmonic / gz-sim8) System plugin that moves a
// model along a smooth cubic (Catmull-Rom / cubic-Hermite) spline defined by a
// set of world-XY waypoints, at a configured constant speed. This mirrors the
// DynaBARN dynamic-obstacle motion model, where obstacles follow cubic-polynomial
// paths through control points.
//
// Motion is applied via a velocity command: each PreUpdate the plugin computes
// the target point on the arc-length-parameterized path for the current sim
// time and drives the model there with:
//     v = speed * unit_tangent  +  k * (path_point - current_pos)   (clamped)
// The result is written to the model's LinearVelocityCmd component, which the
// physics system consumes. Motion is kept planar (no Z, no angular command).

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <gz/plugin/Register.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>

namespace barn
{
namespace sim
{
/// \brief One densely-sampled point along the spline, tagged with the
/// cumulative arc length from the path start and the local unit tangent.
struct PathSample
{
  gz::math::Vector2d pos;      ///< World XY position of the sample.
  gz::math::Vector2d tangent;  ///< Unit tangent (direction of travel).
  double arcLen{0.0};          ///< Cumulative arc length from path start.
};

/// \brief System that drives a model along a cubic spline of waypoints.
class DynamicObstacle
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  // ISystemConfigure
  public: void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  // ISystemPreUpdate
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Build the arc-length-parameterized Catmull-Rom spline samples
  /// from the raw control points.
  private: void BuildSpline();

  /// \brief Look up the path point and unit tangent at a given arc length.
  /// \param[in] _s Arc length (clamped to [0, totalLength]).
  /// \param[out] _pos Interpolated position.
  /// \param[out] _tangent Unit tangent at that position.
  private: void SampleAt(double _s, gz::math::Vector2d &_pos,
      gz::math::Vector2d &_tangent) const;

  /// \brief The model this plugin controls.
  private: gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief Raw control points (world XY) read from SDF.
  private: std::vector<gz::math::Vector2d> waypoints;

  /// \brief Densely-sampled spline used for arc-length lookup.
  private: std::vector<PathSample> samples;

  /// \brief Total arc length of the spline.
  private: double totalLength{0.0};

  /// \brief Target constant speed along the path (m/s).
  private: double speed{1.0};

  /// \brief If true, ping-pong back and forth; if false, stop at the end.
  private: bool loop{true};

  /// \brief Proportional gain on the position-correction term.
  private: double gain{2.0};

  /// \brief Maximum magnitude of the position-correction velocity (m/s).
  private: double maxCorrection{2.0};

  /// \brief Samples generated per spline segment.
  private: static constexpr int kSamplesPerSegment{50};

  /// \brief Whether Configure succeeded and PreUpdate should run.
  private: bool valid{false};
};

//////////////////////////////////////////////////
void DynamicObstacle::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    gzerr << "[DynamicObstacle] Plugin must be attached to a model entity. "
          << "Disabling." << std::endl;
    return;
  }

  // Non-const view of the SDF so we can iterate child elements.
  auto sdf = _sdf->Clone();

  if (sdf->HasElement("speed"))
    this->speed = sdf->Get<double>("speed");
  if (sdf->HasElement("loop"))
    this->loop = sdf->Get<bool>("loop");
  if (sdf->HasElement("gain"))
    this->gain = sdf->Get<double>("gain");
  if (sdf->HasElement("max_correction"))
    this->maxCorrection = sdf->Get<double>("max_correction");

  // Collect all <waypoint>x y</waypoint> elements in order.
  if (sdf->HasElement("waypoint"))
  {
    for (auto wp = sdf->GetElement("waypoint"); wp;
         wp = wp->GetNextElement("waypoint"))
    {
      auto v = wp->Get<gz::math::Vector2d>();
      this->waypoints.push_back(v);
    }
  }

  if (this->waypoints.size() < 2u)
  {
    gzerr << "[DynamicObstacle] Need at least 2 <waypoint> elements, got "
          << this->waypoints.size() << ". Disabling." << std::endl;
    return;
  }

  this->BuildSpline();
  if (this->totalLength <= 0.0 || this->samples.size() < 2u)
  {
    gzerr << "[DynamicObstacle] Degenerate path (zero length). Disabling."
          << std::endl;
    return;
  }

  this->valid = true;
  gzmsg << "[DynamicObstacle] Configured for model '"
        << this->model.Name(_ecm) << "': " << this->waypoints.size()
        << " waypoints, length=" << this->totalLength << " m, speed="
        << this->speed << " m/s, loop=" << (this->loop ? "true" : "false")
        << std::endl;
}

//////////////////////////////////////////////////
void DynamicObstacle::BuildSpline()
{
  this->samples.clear();

  const std::size_t n = this->waypoints.size();
  double cumLen = 0.0;
  gz::math::Vector2d prevPos = this->waypoints.front();
  bool first = true;

  // Iterate over each Catmull-Rom segment [P1, P2], using neighbours P0, P3.
  // Endpoint neighbours are duplicated (clamped) so the curve passes through
  // and terminates at the first/last control points.
  for (std::size_t i = 0; i + 1 < n; ++i)
  {
    const gz::math::Vector2d &p1 = this->waypoints[i];
    const gz::math::Vector2d &p2 = this->waypoints[i + 1];
    const gz::math::Vector2d &p0 = (i == 0) ? p1 : this->waypoints[i - 1];
    const gz::math::Vector2d &p3 =
        (i + 2 < n) ? this->waypoints[i + 2] : p2;

    // Catmull-Rom basis (tension 0.5) -> degree-3 (cubic) polynomial segment.
    for (int k = 0; k < kSamplesPerSegment; ++k)
    {
      const double u = static_cast<double>(k) / (kSamplesPerSegment - 1);
      const double u2 = u * u;
      const double u3 = u2 * u;

      const gz::math::Vector2d pos = 0.5 * (
          2.0 * p1 +
          (-p0 + p2) * u +
          (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2 +
          (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u3);

      // Derivative w.r.t. u gives the (unnormalized) tangent direction.
      gz::math::Vector2d tan = 0.5 * (
          (-p0 + p2) +
          (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * (2.0 * u) +
          (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * (3.0 * u2));
      if (tan.Length() > 1e-9)
        tan.Normalize();

      // Skip the duplicated first sample of each interior segment so arc
      // length accumulates continuously across segment joins.
      if (!first && k == 0)
        continue;

      if (!first)
        cumLen += (pos - prevPos).Length();
      first = false;
      prevPos = pos;

      PathSample s;
      s.pos = pos;
      s.tangent = tan;
      s.arcLen = cumLen;
      this->samples.push_back(s);
    }
  }

  this->totalLength = this->samples.empty() ? 0.0 : this->samples.back().arcLen;
}

//////////////////////////////////////////////////
void DynamicObstacle::SampleAt(double _s, gz::math::Vector2d &_pos,
    gz::math::Vector2d &_tangent) const
{
  _s = std::clamp(_s, 0.0, this->totalLength);

  // Linear scan for the bracketing samples (paths are small).
  std::size_t i = 0;
  for (; i + 1 < this->samples.size(); ++i)
  {
    if (this->samples[i + 1].arcLen >= _s)
      break;
  }
  const PathSample &a = this->samples[i];
  const PathSample &b =
      this->samples[std::min(i + 1, this->samples.size() - 1)];

  const double seg = b.arcLen - a.arcLen;
  const double frac = (seg > 1e-9) ? (_s - a.arcLen) / seg : 0.0;

  _pos = a.pos + (b.pos - a.pos) * frac;

  // Prefer the direction of the local segment; fall back to stored tangent.
  gz::math::Vector2d dir = b.pos - a.pos;
  if (dir.Length() > 1e-9)
    _tangent = dir.Normalized();
  else
    _tangent = a.tangent;
}

//////////////////////////////////////////////////
void DynamicObstacle::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (!this->valid || _info.paused)
    return;

  // dt of zero carries no time information; nothing to servo toward.
  const double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt <= 0.0)
    return;

  // Elapsed simulation time drives the arc-length position.
  const double t = std::chrono::duration<double>(_info.simTime).count();
  const double travelled = this->speed * t;

  // Map elapsed distance onto the path, with ping-pong (loop) or clamp (stop).
  double s = 0.0;
  double dirSign = 1.0;
  if (this->loop)
  {
    const double period = 2.0 * this->totalLength;
    double phase = std::fmod(travelled, period);
    if (phase < 0.0)
      phase += period;
    if (phase <= this->totalLength)
    {
      s = phase;
      dirSign = 1.0;
    }
    else
    {
      s = period - phase;   // returning leg
      dirSign = -1.0;
    }
  }
  else
  {
    s = std::min(travelled, this->totalLength);
    // Stop feed-forward once the end is reached.
    dirSign = (travelled >= this->totalLength) ? 0.0 : 1.0;
  }

  gz::math::Vector2d targetPos, tangent;
  this->SampleAt(s, targetPos, tangent);

  // Current world position of the model (planar XY).
  const gz::math::Pose3d pose =
      gz::sim::worldPose(this->model.Entity(), _ecm);
  const gz::math::Vector2d curPos(pose.Pos().X(), pose.Pos().Y());

  // Feedforward along the path + clamped proportional position correction.
  gz::math::Vector2d vel = this->speed * dirSign * tangent;

  gz::math::Vector2d correction = this->gain * (targetPos - curPos);
  if (correction.Length() > this->maxCorrection)
    correction = correction.Normalized() * this->maxCorrection;
  vel += correction;

  // Planar velocity command: no Z, no angular.
  const gz::math::Vector3d cmd(vel.X(), vel.Y(), 0.0);

  auto *comp =
      _ecm.Component<gz::sim::components::LinearVelocityCmd>(
          this->model.Entity());
  if (comp == nullptr)
  {
    _ecm.CreateComponent(this->model.Entity(),
        gz::sim::components::LinearVelocityCmd(cmd));
  }
  else
  {
    comp->Data() = cmd;
  }
}

}  // namespace sim
}  // namespace barn

// Register the plugin and a stable string alias.
GZ_ADD_PLUGIN(
    barn::sim::DynamicObstacle,
    gz::sim::System,
    barn::sim::DynamicObstacle::ISystemConfigure,
    barn::sim::DynamicObstacle::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(barn::sim::DynamicObstacle, "barn::sim::DynamicObstacle")
