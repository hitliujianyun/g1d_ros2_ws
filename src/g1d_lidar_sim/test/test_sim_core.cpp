#include <gtest/gtest.h>

#include <cmath>

#include "g1d_lidar_sim/sim_core.hpp"

using namespace g1d_lidar_sim;

TEST(SimCore, RaycastBoxFloorCeiling)
{
  World w;
  w.boxes.push_back({Eigen::Vector3d(2.0, -1.0, 0.0), Eigen::Vector3d(2.2, 1.0, 2.0)});
  w.ceiling = 2.8;
  const Eigen::Vector3d o(0.0, 0.0, 1.0);
  auto t = w.raycast(o, Eigen::Vector3d(1.0, 0.0, 0.0), 40.0);
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(*t, 2.0, 1e-9);
  t = w.raycast(o, Eigen::Vector3d(0.0, 0.0, -1.0), 40.0);
  EXPECT_NEAR(*t, 1.0, 1e-9);
  t = w.raycast(o, Eigen::Vector3d(0.0, 0.0, 1.0), 40.0);
  EXPECT_NEAR(*t, 1.8, 1e-9);
  EXPECT_FALSE(w.raycast(o, Eigen::Vector3d(-1.0, 0.0, 0.0), 40.0).has_value());
  // origin inside a box: exit distance
  auto inside = rayBox(Eigen::Vector3d(2.1, 0.0, 1.0), Eigen::Vector3d(1.0, 0.0, 0.0),
                       w.boxes[0]);
  EXPECT_NEAR(*inside, 0.1, 1e-9);
}

TEST(SimCore, ScanDirectionsUnitAndInFov)
{
  const double lo = -7.0 * M_PI / 180.0, hi = 52.0 * M_PI / 180.0;
  double min_el = 10.0, max_el = -10.0;
  for (uint64_t k = 0; k < 20000; ++k) {
    const auto d = scanDirection(k, lo, hi);
    EXPECT_NEAR(d.norm(), 1.0, 1e-9);
    const double el = std::asin(d.z());
    min_el = std::min(min_el, el);
    max_el = std::max(max_el, el);
  }
  EXPECT_GE(min_el, lo - 1e-9);
  EXPECT_LE(max_el, hi + 1e-9);
  EXPECT_LT(min_el, lo + 0.01);
  EXPECT_GT(max_el, hi - 0.01);
}

TEST(SimCore, TrajectoryArcAndAccel)
{
  Trajectory tr;
  tr.add({0.0, 0.0, 0.0, 0.0, 0.5, 0.5});
  tr.add({1.0, 0.0, 0.0, 0.0, 0.7, 0.5});   // after t = 1 the second sample is the base
  const auto s = tr.at(1.5);
  EXPECT_NEAR(s.yaw, 0.25, 1e-12);
  const auto a = tr.at(0.5);
  EXPECT_NEAR(a.yaw, 0.25, 1e-12);
  EXPECT_NEAR(a.x, std::sin(0.25), 1e-12);
  EXPECT_NEAR(a.y, 1.0 - std::cos(0.25), 1e-12);
  double alon = 0.0, alpha = 0.0;
  tr.accel(0.5, alon, alpha);
  EXPECT_NEAR(alon, 0.2, 1e-12);
  EXPECT_NEAR(alpha, 0.0, 1e-12);
}

TEST(SimCore, ImuGravityAndCentripetalUpsideDown)
{
  Eigen::Isometry3d base_T_imu = Eigen::Isometry3d::Identity();
  base_T_imu.translate(Eigen::Vector3d(0.0, 0.0, 1.3));
  base_T_imu.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));   // upside down
  auto r = imuFromPlanar(0.0, 0.0, 0.0, 0.0, base_T_imu);
  EXPECT_NEAR(r.acc.z(), -9.81, 1e-9);       // gravity reaction points down in the flipped frame
  EXPECT_NEAR(r.acc.norm(), 9.81, 1e-9);
  r = imuFromPlanar(0.5, 0.5, 0.0, 0.0, base_T_imu);
  EXPECT_NEAR(r.gyr.z(), -0.5, 1e-9);        // yaw rate appears negated about the flipped z
  EXPECT_NEAR(r.acc.y(), -0.25, 1e-9);       // centripetal v*w to the left (base +y -> imu -y)
}
