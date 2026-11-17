// utlidar_adapter: PointCloud2 (Unitree /utlidar/cloud_livox_mid360 when the Mid-360S is owned by
// the Unitree service) -> Livox CustomMsg for FAST_LIO.
// The per-point time field is detected by name (time, timestamp, t, offset_time) and unit (auto
// from the spread within the frame, or given). Without a time field the points are spread
// uniformly over `scan_period` (motion de-skew then becomes approximate: reduce speed).
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "g1d_lidar/point_time.hpp"

using livox_ros_driver2::msg::CustomMsg;
using livox_ros_driver2::msg::CustomPoint;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace
{
const PointField * findField(const PointCloud2 & c, const std::vector<std::string> & names)
{
  for (const auto & n : names) {
    for (const auto & f : c.fields) {
      if (f.name == n) {return &f;}
    }
  }
  return nullptr;
}

double readValue(const uint8_t * p, uint8_t type)
{
  switch (type) {
    case PointField::INT8: {int8_t v; std::memcpy(&v, p, 1); return v;}
    case PointField::UINT8: return *p;
    case PointField::INT16: {int16_t v; std::memcpy(&v, p, 2); return v;}
    case PointField::UINT16: {uint16_t v; std::memcpy(&v, p, 2); return v;}
    case PointField::INT32: {int32_t v; std::memcpy(&v, p, 4); return v;}
    case PointField::UINT32: {uint32_t v; std::memcpy(&v, p, 4); return v;}
    case PointField::FLOAT32: {float v; std::memcpy(&v, p, 4); return v;}
    case PointField::FLOAT64: {double v; std::memcpy(&v, p, 8); return v;}
    default: return 0.0;
  }
}
}  // namespace

class UtlidarAdapter : public rclcpp::Node
{
public:
  explicit UtlidarAdapter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("utlidar_adapter", options)
  {
    time_field_ = declare_parameter<std::string>("time_field", "");
    time_unit_ = declare_parameter<std::string>("time_unit", "auto");
    scan_period_ = declare_parameter<double>("scan_period", 0.1);
    n_lines_ = static_cast<int>(declare_parameter<int>("n_lines", 4));
    frame_override_ = declare_parameter<std::string>("frame_id", "");
    pub_ = create_publisher<CustomMsg>("output", rclcpp::QoS(10));
    sub_ = create_subscription<PointCloud2>(
      "input", rclcpp::SensorDataQoS(), [this](PointCloud2::UniquePtr m) {onCloud(*m);});
  }

private:
  void onCloud(const PointCloud2 & c)
  {
    const PointField * fx = findField(c, {"x"});
    const PointField * fy = findField(c, {"y"});
    const PointField * fz = findField(c, {"z"});
    if (!fx || !fy || !fz) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cloud without x/y/z fields");
      return;
    }
    const PointField * ft = time_field_.empty() ?
      findField(c, {"time", "timestamp", "t", "offset_time"}) : findField(c, {time_field_});
    const PointField * fi = findField(c, {"intensity", "reflectivity"});
    const PointField * fl = findField(c, {"ring", "line"});
    const PointField * ftag = findField(c, {"tag"});
    const size_t n = static_cast<size_t>(c.width) * c.height;
    std::vector<CustomPoint> pts;
    std::vector<double> times;
    pts.reserve(n);
    times.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      const uint8_t * base = c.data.data() + i * c.point_step;
      CustomPoint p;
      p.x = static_cast<float>(readValue(base + fx->offset, fx->datatype));
      p.y = static_cast<float>(readValue(base + fy->offset, fy->datatype));
      p.z = static_cast<float>(readValue(base + fz->offset, fz->datatype));
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
      if (p.x == 0.0f && p.y == 0.0f && p.z == 0.0f) {continue;}
      p.reflectivity = fi ? static_cast<uint8_t>(std::clamp(readValue(base + fi->offset, fi->datatype), 0.0, 255.0)) : 0;
      p.tag = ftag ? static_cast<uint8_t>(readValue(base + ftag->offset, ftag->datatype)) & 0x30 : 0;
      p.line = static_cast<uint8_t>(
        (fl ? static_cast<int>(readValue(base + fl->offset, fl->datatype)) : static_cast<int>(pts.size())) % n_lines_);
      if (ft) {times.push_back(readValue(base + ft->offset, ft->datatype));}
      pts.push_back(p);
    }
    if (pts.empty()) {return;}
    const uint64_t stamp_ns = static_cast<uint64_t>(rclcpp::Time(c.header.stamp).nanoseconds());
    uint64_t timebase = stamp_ns;
    std::vector<uint32_t> off;
    if (ft) {
      double t0_s = 0.0;
      off = g1d_lidar::offsetsNs(times, g1d_lidar::timeUnitScale(time_unit_), t0_s);
      if (t0_s > 1e8) {                         // absolute times (epoch)
        timebase = static_cast<uint64_t>(std::llround(t0_s * 1e9));
      }
    } else {
      RCLCPP_WARN_ONCE(get_logger(), "cloud has no per-point time field: uniform %.3f s spread", scan_period_);
      off = g1d_lidar::uniformOffsetsNs(pts.size(), scan_period_);
    }
    auto out = std::make_unique<CustomMsg>();
    out->header = c.header;
    if (!frame_override_.empty()) {out->header.frame_id = frame_override_;}
    out->header.stamp = rclcpp::Time(static_cast<int64_t>(timebase));
    out->timebase = timebase;
    for (size_t i = 0; i < pts.size(); ++i) {pts[i].offset_time = off[i];}
    out->points = std::move(pts);
    out->point_num = static_cast<uint32_t>(out->points.size());
    pub_->publish(std::move(out));
  }

  std::string time_field_, time_unit_, frame_override_;
  double scan_period_{0.1};
  int n_lines_{4};
  rclcpp::Publisher<CustomMsg>::SharedPtr pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UtlidarAdapter>());
  rclcpp::shutdown();
  return 0;
}
