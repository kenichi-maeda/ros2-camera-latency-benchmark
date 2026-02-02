#include "camera_latency_benchmark/latency_probe_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <numeric>

#include "rmw/qos_profiles.h"

namespace camera_latency_benchmark
{

namespace
{

double percentile_from_sorted(const std::vector<double> & sorted, double p)
{
  if (sorted.empty()) {
    return 0.0;
  }
  const double clamped = std::max(0.0, std::min(1.0, p));
  const size_t idx = static_cast<size_t>(clamped * static_cast<double>(sorted.size() - 1));
  return sorted[idx];
}

}  // namespace

LatencyProbeNode::LatencyProbeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("latency_probe", options),
  frame_count_(0)
{
  transport_ = declare_parameter<std::string>("transport", "raw");
  input_topic_ = declare_parameter<std::string>("input_topic", "image");
  qos_depth_ = declare_parameter<int>("qos_depth", 10);
  qos_reliability_ = declare_parameter<std::string>("qos_reliability", "best_effort");
  stats_every_ = declare_parameter<int>("stats_every", 100);
  stats_window_ = declare_parameter<int>("stats_window", 1000);
  csv_path_ = declare_parameter<std::string>("csv_path", "");

  if (!csv_path_.empty()) {
    bool write_header = true;
    std::ifstream existing(csv_path_, std::ios::binary);
    if (existing.good()) {
      existing.seekg(0, std::ios::end);
      if (existing.tellg() > 0) {
        write_header = false;
      }
    }
    csv_.open(csv_path_, std::ios::app);
    if (csv_.is_open() && write_header) {
      csv_ << "stamp_ns,latency_ms\n";
    }
  }

  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = static_cast<size_t>(std::max(1, qos_depth_));
  if (qos_reliability_ == "reliable") {
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  } else {
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }

  subscriber_ = image_transport::create_subscription(
    this,
    input_topic_,
    std::bind(&LatencyProbeNode::on_image, this, std::placeholders::_1),
    transport_,
    qos);
}

void LatencyProbeNode::on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  const rclcpp::Time now = get_clock()->now();
  const rclcpp::Time stamp(msg->header.stamp);
  const double latency_ms = (now - stamp).seconds() * 1000.0;

  latencies_ms_.push_back(latency_ms);
  if (static_cast<int>(latencies_ms_.size()) > stats_window_) {
    latencies_ms_.pop_front();
  }

  if (csv_.is_open()) {
    csv_ << stamp.nanoseconds() << ',' << latency_ms << '\n';
  }

  ++frame_count_;
  if (stats_every_ > 0 && (frame_count_ % static_cast<uint64_t>(stats_every_) == 0)) {
    report_stats();
  }
}

void LatencyProbeNode::report_stats()
{
  if (latencies_ms_.empty()) {
    return;
  }

  std::vector<double> sorted(latencies_ms_.begin(), latencies_ms_.end());
  std::sort(sorted.begin(), sorted.end());

  const double sum = std::accumulate(sorted.begin(), sorted.end(), 0.0);
  const double mean = sum / static_cast<double>(sorted.size());
  const double p50 = percentile_from_sorted(sorted, 0.50);
  const double p95 = percentile_from_sorted(sorted, 0.95);
  const double p99 = percentile_from_sorted(sorted, 0.99);

  RCLCPP_INFO(
    get_logger(),
    "latency_ms (window=%zu): mean=%.3f p50=%.3f p95=%.3f p99=%.3f min=%.3f max=%.3f",
    sorted.size(),
    mean,
    p50,
    p95,
    p99,
    sorted.front(),
    sorted.back());
}

}  // namespace camera_latency_benchmark

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_latency_benchmark::LatencyProbeNode)
