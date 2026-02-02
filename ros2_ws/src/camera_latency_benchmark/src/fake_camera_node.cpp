#include "camera_latency_benchmark/fake_camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

#include "rmw/qos_profiles.h"

namespace camera_latency_benchmark
{

FakeCameraNode::FakeCameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fake_camera", options),
  frame_count_(0)
{
  width_ = declare_parameter<int>("width", 640);
  height_ = declare_parameter<int>("height", 480);
  fps_ = declare_parameter<double>("fps", 30.0);
  frame_id_ = declare_parameter<std::string>("frame_id", "camera");
  encoding_ = declare_parameter<std::string>("encoding", "rgb8");
  output_topic_ = declare_parameter<std::string>("output_topic", "image");
  qos_depth_ = declare_parameter<int>("qos_depth", 10);
  qos_reliability_ = declare_parameter<std::string>("qos_reliability", "best_effort");

  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = static_cast<size_t>(std::max(1, qos_depth_));
  if (qos_reliability_ == "reliable") {
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  } else {
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }

  publisher_ = image_transport::create_publisher(this, output_topic_, qos);

  initialize_message();

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, fps_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&FakeCameraNode::on_timer, this));
}

void FakeCameraNode::initialize_message()
{
  msg_.height = static_cast<uint32_t>(height_);
  msg_.width = static_cast<uint32_t>(width_);
  msg_.encoding = encoding_;
  msg_.is_bigendian = 0;
  msg_.step = static_cast<sensor_msgs::msg::Image::_step_type>(width_ * 3);
  msg_.data.resize(static_cast<size_t>(height_) * msg_.step);

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const size_t idx = static_cast<size_t>(y) * msg_.step + static_cast<size_t>(x) * 3;
      msg_.data[idx + 0] = static_cast<uint8_t>(x % 256);
      msg_.data[idx + 1] = static_cast<uint8_t>(y % 256);
      msg_.data[idx + 2] = static_cast<uint8_t>((x + y) % 256);
    }
  }
}

void FakeCameraNode::update_pattern()
{
  const uint8_t delta = static_cast<uint8_t>(frame_count_ % 256);
  const size_t stride = msg_.step;
  for (int y = 0; y < height_; y += 8) {
    const size_t idx = static_cast<size_t>(y) * stride;
    if (idx + 2 < msg_.data.size()) {
      msg_.data[idx + 2] = delta;
    }
  }
}

void FakeCameraNode::on_timer()
{
  msg_.header.stamp = get_clock()->now();
  msg_.header.frame_id = frame_id_;
  update_pattern();
  publisher_.publish(msg_);
  ++frame_count_;
}

}  // namespace camera_latency_benchmark

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_latency_benchmark::FakeCameraNode)
