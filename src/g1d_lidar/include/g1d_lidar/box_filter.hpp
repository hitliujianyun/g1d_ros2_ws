// Self-return filtering: points inside robot body boxes are removed before FAST_LIO / costmaps.
// Header-only, depends on Eigen only (unit tested without ROS).
#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace g1d_lidar
{

/// Axis-aligned box in its own frame (e.g. head_link, base_link).
struct BodyBox
{
  std::string name;
  std::string frame;
  Eigen::Vector3f min{Eigen::Vector3f::Zero()};
  Eigen::Vector3f max{Eigen::Vector3f::Zero()};
};

/// A box with the transform that maps lidar-frame points into the box frame.
struct PlacedBox
{
  Eigen::Isometry3f box_T_lidar{Eigen::Isometry3f::Identity()};
  Eigen::Vector3f min{Eigen::Vector3f::Zero()};
  Eigen::Vector3f max{Eigen::Vector3f::Zero()};

  bool contains(const Eigen::Vector3f & p_lidar) const
  {
    const Eigen::Vector3f p = box_T_lidar * p_lidar;
    return (p.array() >= min.array()).all() && (p.array() <= max.array()).all();
  }
};

/// True if the point must be removed (inside any box or closer than min_range).
inline bool isSelfReturn(
  const Eigen::Vector3f & p, const std::vector<PlacedBox> & boxes, float min_range)
{
  if (min_range > 0.0f && p.squaredNorm() < min_range * min_range) {
    return true;
  }
  for (const auto & b : boxes) {
    if (b.contains(p)) {
      return true;
    }
  }
  return false;
}

/// Parses a flat [xmin, xmax, ymin, ymax, zmin, zmax] list (the g1d_ws convention).
inline bool parseBox(const std::vector<double> & v, BodyBox & out)
{
  if (v.size() != 6 || v[0] > v[1] || v[2] > v[3] || v[4] > v[5]) {
    return false;
  }
  out.min = Eigen::Vector3f(static_cast<float>(v[0]), static_cast<float>(v[2]), static_cast<float>(v[4]));
  out.max = Eigen::Vector3f(static_cast<float>(v[1]), static_cast<float>(v[3]), static_cast<float>(v[5]));
  return true;
}

}  // namespace g1d_lidar
