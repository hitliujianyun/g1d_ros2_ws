// Mid-360 + IMU simulation core (ported from g1d_ws@068e6dd src/sim): box world ray casting,
// non-repetitive scan pattern, planar ground-truth trajectory and IMU readings. Eigen only.
#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

namespace g1d_lidar_sim
{

struct Box3
{
  Eigen::Vector3d min;
  Eigen::Vector3d max;
};

/// Ray / axis-aligned box intersection distance (> eps), if any.
inline std::optional<double> rayBox(
  const Eigen::Vector3d & o, const Eigen::Vector3d & d, const Box3 & b, double eps = 1e-6)
{
  double t0 = -1e30, t1 = 1e30;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(d[i]) < 1e-12) {
      if (o[i] < b.min[i] || o[i] > b.max[i]) {return std::nullopt;}
      continue;
    }
    double a = (b.min[i] - o[i]) / d[i];
    double c = (b.max[i] - o[i]) / d[i];
    if (a > c) {std::swap(a, c);}
    t0 = std::max(t0, a);
    t1 = std::min(t1, c);
    if (t0 > t1) {return std::nullopt;}
  }
  if (t1 < eps) {return std::nullopt;}
  return t0 > eps ? t0 : t1;
}

/// Indoor world: floor at z = 0, optional ceiling, axis-aligned boxes.
struct World
{
  std::vector<Box3> boxes;
  std::optional<double> ceiling;

  std::optional<double> raycast(const Eigen::Vector3d & o, const Eigen::Vector3d & d, double max_range) const
  {
    double best = max_range;
    bool hit = false;
    for (const auto & b : boxes) {
      auto t = rayBox(o, d, b);
      if (t && *t < best) {best = *t; hit = true;}
    }
    if (d.z() < -1e-9) {
      const double t = -o.z() / d.z();
      if (t > 1e-6 && t < best) {best = t; hit = true;}
    }
    if (ceiling && d.z() > 1e-9) {
      const double t = (*ceiling - o.z()) / d.z();
      if (t > 1e-6 && t < best) {best = t; hit = true;}
    }
    return hit ? std::optional<double>(best) : std::nullopt;
  }
};

/// Low-discrepancy (rosette-like) scan direction number k in the sensor frame: azimuth by the
/// golden ratio, elevation uniform in sin(el) within [fov_min, fov_max] (rad).
inline Eigen::Vector3d scanDirection(uint64_t k, double fov_min, double fov_max)
{
  constexpr double kGolden = 0.6180339887498949;
  constexpr double kSqrt2Frac = 0.4142135623730951;
  const double sin_lo = std::sin(fov_min), sin_hi = std::sin(fov_max);
  const double az = 2.0 * M_PI * std::fmod(k * kGolden, 1.0);
  const double u = std::fmod(k * kSqrt2Frac + 0.5 * std::fmod(k * 0.000137, 1.0), 1.0);
  const double sel = sin_lo + (sin_hi - sin_lo) * u;
  const double cel = std::sqrt(std::max(0.0, 1.0 - sel * sel));
  return Eigen::Vector3d(cel * std::cos(az), cel * std::sin(az), sel);
}

struct GtSample
{
  double t{0.0}, x{0.0}, y{0.0}, yaw{0.0}, v{0.0}, w{0.0};
};

/// Planar base trajectory from ground-truth samples; constant (v, w) arcs between samples.
class Trajectory
{
public:
  void add(const GtSample & s)
  {
    if (!s_.empty() && s.t <= s_.back().t) {return;}
    s_.push_back(s);
  }
  void prune(double t_keep) {while (s_.size() > 3 && s_[1].t < t_keep) {s_.pop_front();}}
  bool empty() const {return s_.empty();}
  double front() const {return s_.front().t;}
  double back() const {return s_.back().t;}

  GtSample at(double t) const
  {
    const GtSample & a = s_[index(t)];
    GtSample r = a;
    const double dt = t - a.t;
    if (std::abs(a.w) < 1e-9) {
      r.x += a.v * dt * std::cos(a.yaw);
      r.y += a.v * dt * std::sin(a.yaw);
    } else {
      r.x += a.v / a.w * (std::sin(a.yaw + a.w * dt) - std::sin(a.yaw));
      r.y -= a.v / a.w * (std::cos(a.yaw + a.w * dt) - std::cos(a.yaw));
    }
    r.yaw = a.yaw + a.w * dt;
    r.t = t;
    return r;
  }

  /// Longitudinal acceleration and angular acceleration by central differences of the samples.
  void accel(double t, double & a_lon, double & alpha) const
  {
    const size_t i = index(t);
    const size_t lo = i > 0 ? i - 1 : i;
    const size_t hi = std::min(i + 1, s_.size() - 1);
    const double dt = s_[hi].t - s_[lo].t;
    a_lon = dt > 1e-6 ? (s_[hi].v - s_[lo].v) / dt : 0.0;
    alpha = dt > 1e-6 ? (s_[hi].w - s_[lo].w) / dt : 0.0;
  }

  Eigen::Isometry3d pose(double t) const
  {
    const GtSample s = at(t);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translate(Eigen::Vector3d(s.x, s.y, 0.0));
    T.rotate(Eigen::AngleAxisd(s.yaw, Eigen::Vector3d::UnitZ()));
    return T;
  }

private:
  size_t index(double t) const
  {
    auto it = std::upper_bound(s_.begin(), s_.end(), t, [](double v, const GtSample & s) {return v < s.t;});
    return it == s_.begin() ? 0 : static_cast<size_t>(it - s_.begin() - 1);
  }
  std::deque<GtSample> s_;
};

struct ImuReading
{
  Eigen::Vector3d gyr;
  Eigen::Vector3d acc;   // specific force, m/s^2
};

/// IMU readings of a sensor rigidly mounted at base_T_imu on a planar base moving with forward
/// speed v, yaw rate w, longitudinal acceleration a_lon and yaw acceleration alpha.
inline ImuReading imuFromPlanar(
  double v, double w, double a_lon, double alpha, const Eigen::Isometry3d & base_T_imu,
  double g = 9.81)
{
  const Eigen::Vector3d omega(0.0, 0.0, w), alpha_v(0.0, 0.0, alpha);
  const Eigen::Vector3d r = base_T_imu.translation();
  const Eigen::Vector3d a_center(a_lon, v * w, 0.0);
  const Eigen::Vector3d a = a_center + alpha_v.cross(r) + omega.cross(omega.cross(r));
  const Eigen::Matrix3d Rt = base_T_imu.linear().transpose();
  return {Rt * omega, Rt * (a + Eigen::Vector3d(0.0, 0.0, g))};
}

}  // namespace g1d_lidar_sim
