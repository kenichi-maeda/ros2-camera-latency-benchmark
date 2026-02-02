#ifndef CAMERA_LATENCY_BENCHMARK__FAKE_CAMERA_NODE_HPP_
#define CAMERA_LATENCY_BENCHMARK__FAKE_CAMERA_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_latency_benchmark
{

class FakeCameraNode : public rclcpp::Node
{
public:
  explicit FakeCameraNode(const rclcpp::NodeOptions & options);

private:
  void on_timer();
  void initialize_message();
  void update_pattern();

  image_transport::Publisher publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Image msg_;
  std::string frame_id_;
  std::string encoding_;
  std::string output_topic_;

  int width_;
  int height_;
  double fps_;
  int qos_depth_;
  std::string qos_reliability_;

  uint64_t frame_count_;
};

}  // namespace camera_latency_benchmark

#endif  // CAMERA_LATENCY_BENCHMARK__FAKE_CAMERA_NODE_HPP_
