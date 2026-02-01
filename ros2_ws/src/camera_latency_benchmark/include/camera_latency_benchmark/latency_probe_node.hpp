#ifndef CAMERA_LATENCY_BENCHMARK__LATENCY_PROBE_NODE_HPP_
#define CAMERA_LATENCY_BENCHMARK__LATENCY_PROBE_NODE_HPP_

#include <cstdint>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_latency_benchmark
{

class LatencyProbeNode : public rclcpp::Node
{
public:
  explicit LatencyProbeNode(const rclcpp::NodeOptions & options);

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void report_stats();

  image_transport::Subscriber subscriber_;

  std::deque<double> latencies_ms_;
  std::ofstream csv_;

  std::string transport_;
  int qos_depth_;
  std::string qos_reliability_;
  int stats_every_;
  int stats_window_;
  std::string csv_path_;

  uint64_t frame_count_;
};

}  // namespace camera_latency_benchmark

#endif  // CAMERA_LATENCY_BENCHMARK__LATENCY_PROBE_NODE_HPP_
