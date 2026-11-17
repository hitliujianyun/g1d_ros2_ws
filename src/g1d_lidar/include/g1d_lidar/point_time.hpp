// Per-point time handling for PointCloud2 -> Livox CustomMsg conversion (unitree /utlidar/* source).
// Header-only, no ROS dependency.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace g1d_lidar
{

/// Seconds per unit of the time field ("s", "ms", "us", "ns"); 0 for "auto"/unknown.
inline double timeUnitScale(const std::string & unit)
{
  if (unit == "s") {return 1.0;}
  if (unit == "ms") {return 1e-3;}
  if (unit == "us") {return 1e-6;}
  if (unit == "ns") {return 1e-9;}
  return 0.0;
}

/// Guesses the unit from the spread of the time values within one ~0.1 s frame.
inline double guessTimeUnitScale(double span)
{
  if (span <= 0.0) {return 1.0;}
  if (span < 1.0) {return 1.0;}        // seconds (frame spans ~0.1)
  if (span < 1e3) {return 1e-3;}       // ms (~100)
  if (span < 1e6) {return 1e-6;}       // us (~1e5)
  return 1e-9;                         // ns (~1e8)
}

/// Offsets (ns, relative to the earliest point) from raw per-point time values.
/// Values may be absolute (e.g. epoch) or relative; both give the same offsets.
inline std::vector<uint32_t> offsetsNs(const std::vector<double> & t, double scale, double & t0_s)
{
  std::vector<uint32_t> out(t.size(), 0);
  if (t.empty()) {
    t0_s = 0.0;
    return out;
  }
  const auto [mn, mx] = std::minmax_element(t.begin(), t.end());
  if (scale <= 0.0) {
    scale = guessTimeUnitScale(*mx - *mn);
  }
  t0_s = *mn * scale;
  for (size_t i = 0; i < t.size(); ++i) {
    const double ns = (t[i] - *mn) * scale * 1e9;
    out[i] = static_cast<uint32_t>(std::clamp(std::llround(ns), 0LL, 4000000000LL));
  }
  return out;
}

/// Offsets when the cloud has no time field: spread uniformly over the scan period.
inline std::vector<uint32_t> uniformOffsetsNs(size_t n, double period_s)
{
  std::vector<uint32_t> out(n, 0);
  for (size_t i = 0; i < n && n > 1; ++i) {
    out[i] = static_cast<uint32_t>(std::llround(period_s * 1e9 * static_cast<double>(i) / (n - 1)));
  }
  return out;
}

}  // namespace g1d_lidar
