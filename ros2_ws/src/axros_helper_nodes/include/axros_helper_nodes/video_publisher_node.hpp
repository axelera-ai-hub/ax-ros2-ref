// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Declares VideoPublisherNode, which republishes a video file as a ROS2 image
// stream.

#pragma once

#include <string>

#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace axros_helper_nodes
{

class VideoPublisherNode : public rclcpp::Node
{
public:
  explicit VideoPublisherNode(const rclcpp::NodeOptions & options);
  ~VideoPublisherNode();

private:
  void timer_callback();

  // Parameters
  std::string video_path_;
  std::string output_topic_;
  double publish_fps_;
  bool loop_;
  std::string frame_id_;

  // Runtime state
  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double fps_{30.0};
  int frame_count_{0};
};

}  // namespace axros_helper_nodes
