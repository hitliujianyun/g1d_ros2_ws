#include <gtest/gtest.h>

#include <cmath>

#include "g1d_lidar/box_filter.hpp"
#include "g1d_lidar/point_time.hpp"

using g1d_lidar::BodyBox;
using g1d_lidar::PlacedBox;

TEST(BoxFilter, ParseAndContain)
{
  BodyBox b;
  ASSERT_TRUE(g1d_lidar::parseBox({-0.2, 0.2, -0.25, 0.25, -0.95, 0.1}, b));
  EXPECT_FLOAT_EQ(b.min.z(), -0.95f);
  EXPECT_FLOAT_EQ(b.max.y(), 0.25f);
  EXPECT_FALSE(g1d_lidar::parseBox({1, 0, 0, 1, 0, 1}, b));
  EXPECT_FALSE(g1d_lidar::parseBox({0, 1}, b));
}

TEST(BoxFilter, TransformedBoxAndMinRange)
{
  // Box frame = lidar frame rotated by pi about x (upside-down lidar) and shifted 0.5 m in z.
  PlacedBox pb;
  pb.min = Eigen::Vector3f(-0.1f, -0.1f, -0.1f);
  pb.max = Eigen::Vector3f(0.1f, 0.1f, 0.1f);
  Eigen::Isometry3f lidar_T_box = Eigen::Isometry3f::Identity();
  lidar_T_box.translate(Eigen::Vector3f(0.0f, 0.0f, 0.5f));
  lidar_T_box.rotate(Eigen::AngleAxisf(static_cast<float>(M_PI), Eigen::Vector3f::UnitX()));
  pb.box_T_lidar = lidar_T_box.inverse();
  std::vector<PlacedBox> boxes{pb};
  EXPECT_TRUE(g1d_lidar::isSelfReturn(Eigen::Vector3f(0.0f, 0.05f, 0.55f), boxes, 0.0f));
  EXPECT_FALSE(g1d_lidar::isSelfReturn(Eigen::Vector3f(0.0f, 0.0f, 0.75f), boxes, 0.0f));
  EXPECT_TRUE(g1d_lidar::isSelfReturn(Eigen::Vector3f(0.1f, 0.0f, 0.0f), boxes, 0.2f));
  EXPECT_FALSE(g1d_lidar::isSelfReturn(Eigen::Vector3f(3.0f, 0.0f, 0.0f), boxes, 0.2f));
}

TEST(PointTime, UnitsAndOffsets)
{
  EXPECT_DOUBLE_EQ(g1d_lidar::timeUnitScale("us"), 1e-6);
  EXPECT_DOUBLE_EQ(g1d_lidar::timeUnitScale("auto"), 0.0);
  double t0 = 0.0;
  // absolute seconds (epoch) over 0.1 s
  auto off = g1d_lidar::offsetsNs({1.7e9 + 0.05, 1.7e9, 1.7e9 + 0.1}, 0.0, t0);
  EXPECT_NEAR(off[0], 50000000u, 2000u);   // double precision at epoch ~ 0.2 us
  EXPECT_EQ(off[1], 0u);
  EXPECT_NEAR(off[2], 100000000u, 2000u);
  EXPECT_NEAR(t0, 1.7e9, 1e-3);
  // relative microseconds, unit guessed
  off = g1d_lidar::offsetsNs({0.0, 50000.0, 100000.0}, 0.0, t0);
  EXPECT_EQ(off[1], 50000000u);
  // explicit nanoseconds
  off = g1d_lidar::offsetsNs({10.0, 110.0}, 1e-9, t0);
  EXPECT_EQ(off[1], 100u);
  auto u = g1d_lidar::uniformOffsetsNs(3, 0.1);
  EXPECT_EQ(u[2], 100000000u);
  EXPECT_EQ(u[1], 50000000u);
}
