// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Declares SaveAnnotatedNode, which writes annotated detection frames to disk.

#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace axros_helper_nodes
{

class SaveAnnotatedNode : public rclcpp::Node
{
public:
  explicit SaveAnnotatedNode(const rclcpp::NodeOptions & options);
  ~SaveAnnotatedNode() = default;

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    vision_msgs::msg::Detection2DArray>;

  void sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & det_msg);

  cv::Mat draw_detections(
    const cv::Mat & frame,
    const vision_msgs::msg::Detection2DArray & detections);

  // Parameters
  std::string image_topic_;
  std::string detections_topic_;
  std::string output_dir_;
  std::string annotated_topic_;
  int save_every_n_;
  bool save_frames_;
  bool publish_annotated_;

  // Subscribers & sync
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;

  // State
  int frame_count_{0};
  int saved_count_{0};
};

}  // namespace axros_helper_nodes
