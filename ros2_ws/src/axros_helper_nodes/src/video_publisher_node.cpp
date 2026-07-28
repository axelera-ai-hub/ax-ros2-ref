// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Implementation of VideoPublisherNode. Publishes frames from a video file as
// sensor_msgs/Image.

#include "axros_helper_nodes/video_publisher_node.hpp"

#include <chrono>
#include <stdexcept>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace axros_helper_nodes
{

VideoPublisherNode::VideoPublisherNode(const rclcpp::NodeOptions & options)
: Node("video_publisher", options)
{
  // ── Declare + get parameters 
  declare_parameter<std::string>("video_path", "");
  declare_parameter<std::string>("output_topic", "image_raw");
  declare_parameter<double>("publish_fps", 0.0);
  declare_parameter<bool>("loop", true);
  declare_parameter<std::string>("frame_id", "camera");

  video_path_   = get_parameter("video_path").as_string();
  output_topic_ = get_parameter("output_topic").as_string();
  publish_fps_  = get_parameter("publish_fps").as_double();
  loop_         = get_parameter("loop").as_bool();
  frame_id_     = get_parameter("frame_id").as_string();

  // ── Validate 
  if (video_path_.empty()) {
    throw std::invalid_argument("Parameter 'video_path' must not be empty");
  }

  // ── Open video 
  cap_.open(video_path_);
  if (!cap_.isOpened()) {
    throw std::runtime_error("Failed to open video file: " + video_path_);
  }

  // ── Determine FPS 
  double native_fps = cap_.get(cv::CAP_PROP_FPS);
  fps_ = (publish_fps_ > 0.0) ? publish_fps_ : native_fps;
  if (fps_ <= 0.0 || fps_ > 120.0) {
    fps_ = 30.0;
  }

  // ── Log info 
  RCLCPP_INFO(get_logger(),
    "VideoPublisherNode:\n"
    "  video_path:    %s\n"
    "  output_topic:  %s\n"
    "  fps:           %.2f\n"
    "  loop:          %s\n"
    "  resolution:    %.0f x %.0f\n"
    "  total_frames:  %.0f",
    video_path_.c_str(),
    output_topic_.c_str(),
    fps_,
    loop_ ? "true" : "false",
    cap_.get(cv::CAP_PROP_FRAME_WIDTH),
    cap_.get(cv::CAP_PROP_FRAME_HEIGHT),
    cap_.get(cv::CAP_PROP_FRAME_COUNT));

  // ── Publisher 
  pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

  // ── Timer 
  using namespace std::chrono_literals;
  auto period = std::chrono::duration<double>(1.0 / fps_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&VideoPublisherNode::timer_callback, this));
}

VideoPublisherNode::~VideoPublisherNode()
{
  if (cap_.isOpened()) {
    cap_.release();
    RCLCPP_INFO(get_logger(), "VideoCapture released");
  }
}

void VideoPublisherNode::timer_callback()
{
  cv::Mat frame;
  bool ok = cap_.read(frame);

  if (!ok || frame.empty()) {
    if (loop_) {
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      RCLCPP_INFO(get_logger(), "Video looped");
      return;
    } else {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "Video ended, stopping publisher");
      return;
    }
  }

  auto msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    "bgr8",
    frame).toImageMsg();

  msg->header.stamp    = this->now();
  msg->header.frame_id = frame_id_;
  ++frame_count_;

  pub_->publish(*msg);

  if (frame_count_ % 100 == 0) {
    RCLCPP_DEBUG(get_logger(), "Published frame %d", frame_count_);
  }
}

}  // namespace axros_helper_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(axros_helper_nodes::VideoPublisherNode)
