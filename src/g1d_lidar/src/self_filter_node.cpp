// self_filter: removes robot self-returns from Livox CustomMsg clouds before FAST_LIO.
//   input (CustomMsg, e.g. /livox/lidar_raw) -> output (CustomMsg, e.g. /livox/lidar)
// Boxes are given in any TF frame (head_link for the head / torso, base_link for the chassis) and
// are re-placed every frame from TF, so lift and waist motion are followed.
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "g1d_lidar/box_filter.hpp"

using livox_ros_driver2::msg::CustomMsg;

class SelfFilter : public rclcpp::Node
{
public:
  explicit SelfFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("self_filter", options)
  {
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "livox_frame");
    min_range_ = static_cast<float>(declare_parameter<double>("min_range", 0.0));
    const auto names = declare_parameter<std::vector<std::string>>("boxes", std::vector<std::string>{});
    for (const auto & name : names) {
      g1d_lidar::BodyBox b;
      b.name = name;
      b.frame = declare_parameter<std::string>(name + ".frame", "base_link");
      const auto v = declare_parameter<std::vector<double>>(name + ".box", std::vector<double>{});
      if (!g1d_lidar::parseBox(v, b)) {
        throw std::runtime_error("self_filter: box '" + name + "' needs [xmin, xmax, ymin, ymax, zmin, zmax]");
      }
      boxes_.push_back(b);
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<CustomMsg>("output", rclcpp::QoS(10));
    sub_ = create_subscription<CustomMsg>(
      "input", rclcpp::SensorDataQoS(),
      [this](CustomMsg::UniquePtr msg) {onCloud(std::move(msg));});
    RCLCPP_INFO(get_logger(), "self_filter: %zu boxes, min_range %.2f m", boxes_.size(), min_range_);
  }

private:
  void onCloud(CustomMsg::UniquePtr msg)
  {
    const std::string frame = msg->header.frame_id.empty() ? lidar_frame_ : msg->header.frame_id;
    std::vector<g1d_lidar::PlacedBox> placed;
    placed.reserve(boxes_.size());
    for (const auto & b : boxes_) {
      g1d_lidar::PlacedBox pb;
      pb.min = b.min;
      pb.max = b.max;
      try {
        const auto t = tf_buffer_->lookupTransform(b.frame, frame, tf2::TimePointZero);
        pb.box_T_lidar = tf2::transformToEigen(t.transform).cast<float>();
        cache_[b.name] = pb.box_T_lidar;
      } catch (const tf2::TransformException & e) {
        auto it = cache_.find(b.name);
        if (it == cache_.end()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "no TF %s <- %s yet, box '%s' not applied", b.frame.c_str(), frame.c_str(), b.name.c_str());
          continue;
        }
        pb.box_T_lidar = it->second;
      }
      placed.push_back(pb);
    }
    auto out = std::make_unique<CustomMsg>();
    out->header = msg->header;
    out->timebase = msg->timebase;
    out->lidar_id = msg->lidar_id;
    out->rsvd = msg->rsvd;
    out->points.reserve(msg->points.size());
    size_t removed = 0;
    for (const auto & p : msg->points) {
      if (g1d_lidar::isSelfReturn(Eigen::Vector3f(p.x, p.y, p.z), placed, min_range_)) {
        ++removed;
        continue;
      }
      out->points.push_back(p);
    }
    out->point_num = static_cast<uint32_t>(out->points.size());
    removed_total_ += removed;
    points_total_ += msg->points.size();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000, "self_filter removed %.1f %% of points",
      points_total_ ? 100.0 * removed_total_ / points_total_ : 0.0);
    pub_->publish(std::move(out));
  }

  std::string lidar_frame_;
  float min_range_{0.0f};
  std::vector<g1d_lidar::BodyBox> boxes_;
  std::map<std::string, Eigen::Isometry3f> cache_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<CustomMsg>::SharedPtr pub_;
  rclcpp::Subscription<CustomMsg>::SharedPtr sub_;
  size_t removed_total_{0};
  size_t points_total_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelfFilter>());
  rclcpp::shutdown();
  return 0;
}
