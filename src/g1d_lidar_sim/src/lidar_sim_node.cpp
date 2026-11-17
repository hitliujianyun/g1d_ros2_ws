// lidar_sim: Mid-360(S) + built-in IMU simulation following the simulated chassis.
//   /sim/ground_truth (Odometry, world frame, from g1d_sim chassis_sim) + TF base_link -> livox_frame /
//   livox_imu (URDF + joints) -> /livox/lidar_raw (Livox CustomMsg, per-point time, motion
//   distortion) and /livox/imu (sensor_msgs/Imu, acceleration in g like the real device).
// World boxes come as a flat parameter (x0, y0, z0, x1, y1, z1, ...); `body_boxes` (base_link
// frame) emulate returns from the robot itself so the self filter can be tested.
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "g1d_lidar_sim/sim_core.hpp"

using livox_ros_driver2::msg::CustomMsg;
using livox_ros_driver2::msg::CustomPoint;
namespace sim = g1d_lidar_sim;

namespace
{
std::vector<sim::Box3> parseBoxes(const std::vector<double> & v)
{
  if (v.size() % 6 != 0) {
    throw std::runtime_error("box list needs a multiple of 6 values (x0, y0, z0, x1, y1, z1)");
  }
  std::vector<sim::Box3> out;
  for (size_t i = 0; i < v.size(); i += 6) {
    sim::Box3 b;
    b.min = Eigen::Vector3d(std::min(v[i], v[i + 3]), std::min(v[i + 1], v[i + 4]), std::min(v[i + 2], v[i + 5]));
    b.max = Eigen::Vector3d(std::max(v[i], v[i + 3]), std::max(v[i + 1], v[i + 4]), std::max(v[i + 2], v[i + 5]));
    out.push_back(b);
  }
  return out;
}

double stampSec(const builtin_interfaces::msg::Time & t) {return t.sec + t.nanosec * 1e-9;}

builtin_interfaces::msg::Time toStamp(double t)
{
  return rclcpp::Time(static_cast<int64_t>(std::llround(t * 1e9)));
}
}  // namespace

class LidarSim : public rclcpp::Node
{
public:
  explicit LidarSim(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lidar_sim", options), rng_(static_cast<uint32_t>(declare_parameter<int>("seed", 1)))
  {
    world_.boxes = parseBoxes(declare_parameter<std::vector<double>>("world_boxes", std::vector<double>{}));
    const double ceiling = declare_parameter<double>("ceiling_z", 2.8);
    if (ceiling > 0.0) {world_.ceiling = ceiling;}
    body_ = parseBoxes(declare_parameter<std::vector<double>>("body_boxes", std::vector<double>{}));
    pps_ = declare_parameter<double>("points_per_second", 100000.0);
    frame_dt_ = 1.0 / declare_parameter<double>("scan_rate", 10.0);
    imu_dt_ = 1.0 / declare_parameter<double>("imu_rate", 200.0);
    fov_min_ = declare_parameter<double>("fov_min_deg", -7.0) * M_PI / 180.0;
    fov_max_ = declare_parameter<double>("fov_max_deg", 52.0) * M_PI / 180.0;
    max_range_ = declare_parameter<double>("max_range", 40.0);
    min_range_ = declare_parameter<double>("min_range", 0.1);
    range_noise_ = std::normal_distribution<double>(0.0, declare_parameter<double>("range_noise", 0.01));
    gyro_noise_ = std::normal_distribution<double>(0.0, declare_parameter<double>("gyro_noise", 0.002));
    acc_noise_ = std::normal_distribution<double>(0.0, declare_parameter<double>("acc_noise", 0.02));
    const auto gb = declare_parameter<std::vector<double>>("gyro_bias", {0.002, -0.001, 0.0015});
    const auto ab = declare_parameter<std::vector<double>>("acc_bias", {0.02, -0.01, 0.015});
    gyro_bias_ = Eigen::Vector3d(gb.at(0), gb.at(1), gb.at(2));
    acc_bias_ = Eigen::Vector3d(ab.at(0), ab.at(1), ab.at(2));
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "livox_frame");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "livox_imu");
    start_delay_ = declare_parameter<double>("start_delay", 1.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_lidar_ = create_publisher<CustomMsg>("/livox/lidar_raw", rclcpp::QoS(10));
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("/livox/imu", rclcpp::QoS(50));
    sub_gt_ = create_subscription<nav_msgs::msg::Odometry>(
      "/sim/ground_truth", rclcpp::QoS(100), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const auto & q = m->pose.pose.orientation;
        sim::GtSample s;
        s.t = stampSec(m->header.stamp);
        s.x = m->pose.pose.position.x;
        s.y = m->pose.pose.position.y;
        s.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        s.v = m->twist.twist.linear.x;
        s.w = m->twist.twist.angular.z;
        traj_.add(s);
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "lidar_sim: %zu world boxes, %zu body boxes, %.0f pts/s",
      world_.boxes.size(), body_.size(), pps_);
  }

private:
  bool lookup(const std::string & child, Eigen::Isometry3d & out)
  {
    try {
      out = tf2::transformToEigen(tf_buffer_->lookupTransform(base_frame_, child, tf2::TimePointZero).transform);
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for TF %s <- %s", base_frame_.c_str(), child.c_str());
      return false;
    }
  }

  void tick()
  {
    if (traj_.empty() || !lookup(lidar_frame_, base_T_lidar_) || !lookup(imu_frame_, base_T_imu_)) {
      return;
    }
    if (!started_) {
      t_frame_ = traj_.front() + start_delay_;
      t_imu_ = std::max(traj_.front(), t_frame_ - 0.5);
      started_ = true;
    }
    const double t_avail = traj_.back();
    while (t_imu_ <= t_avail) {
      publishImu(t_imu_);
      t_imu_ += imu_dt_;
    }
    while (t_frame_ + frame_dt_ <= t_avail) {
      publishFrame(t_frame_);
      t_frame_ += frame_dt_;
    }
    traj_.prune(std::min(t_imu_, t_frame_) - 1.0);
  }

  void publishImu(double t)
  {
    const auto s = traj_.at(t);
    double a_lon = 0.0, alpha = 0.0;
    traj_.accel(t, a_lon, alpha);
    const auto r = sim::imuFromPlanar(s.v, s.w, a_lon, alpha, base_T_imu_);
    const Eigen::Vector3d gyr = r.gyr + gyro_bias_ + Eigen::Vector3d(gyro_noise_(rng_), gyro_noise_(rng_), gyro_noise_(rng_));
    const Eigen::Vector3d acc = (r.acc + acc_bias_ + Eigen::Vector3d(acc_noise_(rng_), acc_noise_(rng_), acc_noise_(rng_))) / 9.81;
    sensor_msgs::msg::Imu m;
    m.header.stamp = toStamp(t);
    m.header.frame_id = lidar_frame_;   // livox_ros_driver2 convention
    m.angular_velocity.x = gyr.x();
    m.angular_velocity.y = gyr.y();
    m.angular_velocity.z = gyr.z();
    m.linear_acceleration.x = acc.x();
    m.linear_acceleration.y = acc.y();
    m.linear_acceleration.z = acc.z();
    pub_imu_->publish(m);
  }

  void publishFrame(double t0)
  {
    const int n = static_cast<int>(std::llround(pps_ * frame_dt_));
    auto msg = std::make_unique<CustomMsg>();
    msg->header.stamp = toStamp(t0);
    msg->header.frame_id = lidar_frame_;
    msg->timebase = static_cast<uint64_t>(std::llround(t0 * 1e9));
    msg->points.reserve(n);
    const Eigen::Vector3d o_body = base_T_lidar_.translation();
    for (int i = 0; i < n; ++i, ++k_) {
      const double t = t0 + frame_dt_ * i / n;
      const Eigen::Isometry3d T = traj_.pose(t) * base_T_lidar_;
      const Eigen::Vector3d d_s = sim::scanDirection(k_, fov_min_, fov_max_);
      auto hit = world_.raycast(T.translation(), T.linear() * d_s, max_range_);
      const Eigen::Vector3d d_b = base_T_lidar_.linear() * d_s;
      for (const auto & b : body_) {
        auto tb = sim::rayBox(o_body, d_b, b);
        if (tb && (!hit || *tb < *hit)) {hit = tb;}
      }
      if (!hit || *hit < min_range_) {continue;}
      const Eigen::Vector3d p = d_s * (*hit + range_noise_(rng_));
      CustomPoint cp;
      cp.offset_time = static_cast<uint32_t>(std::llround((t - t0) * 1e9));
      cp.x = static_cast<float>(p.x());
      cp.y = static_cast<float>(p.y());
      cp.z = static_cast<float>(p.z());
      cp.reflectivity = 100;
      cp.tag = 0;
      cp.line = static_cast<uint8_t>(k_ % 4);
      msg->points.push_back(cp);
    }
    msg->point_num = static_cast<uint32_t>(msg->points.size());
    pub_lidar_->publish(std::move(msg));
  }

  sim::World world_;
  std::vector<sim::Box3> body_;
  sim::Trajectory traj_;
  double pps_{100000.0}, frame_dt_{0.1}, imu_dt_{0.005}, fov_min_{0.0}, fov_max_{0.0};
  double max_range_{40.0}, min_range_{0.1}, start_delay_{1.0};
  std::mt19937 rng_;
  std::normal_distribution<double> range_noise_, gyro_noise_, acc_noise_;
  Eigen::Vector3d gyro_bias_, acc_bias_;
  std::string base_frame_, lidar_frame_, imu_frame_;
  Eigen::Isometry3d base_T_lidar_{Eigen::Isometry3d::Identity()}, base_T_imu_{Eigen::Isometry3d::Identity()};
  bool started_{false};
  double t_frame_{0.0}, t_imu_{0.0};
  uint64_t k_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<CustomMsg>::SharedPtr pub_lidar_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gt_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSim>());
  rclcpp::shutdown();
  return 0;
}
