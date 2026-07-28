// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Declares LiveInferenceNode, which renders a live camera feed with detection
// and color overlays.

#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace axros_helper_nodes
{

class LiveInferenceNode : public rclcpp::Node
{
public:
  explicit LiveInferenceNode(const rclcpp::NodeOptions & options);
  ~LiveInferenceNode();

private:
  void capture_callback();
  void summary_callback();

  // Display runs off the image stream alone — detections are cached
  // separately so the window stays live even when the detector has nothing
  // to report. The previous design synced both streams via message_filters,
  // which froze the window any time the detector went quiet.
  void display_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void det_cache_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg);

  cv::Mat draw_detections(
    const cv::Mat & frame,
    const vision_msgs::msg::Detection2DArray & detections);

  // Parameters
  std::string camera_device_;
  std::string output_topic_;
  double      fps_;
  std::string frame_id_;
  int         summary_every_s_;
  std::string window_name_;

  // Camera
  cv::VideoCapture cap_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;

  // Regular subscriptions — decoupled so display doesn't depend on detector.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           display_image_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_cache_sub_;

  // Cached latest detections. Stale beyond a short window so overlays
  // don't linger after the tracked object leaves the scene.
  vision_msgs::msg::Detection2DArray::ConstSharedPtr latest_dets_;
  rclcpp::Time                                       latest_dets_time_;
  std::mutex                                         latest_dets_mtx_;
  static constexpr double DET_OVERLAY_STALE_S = 1.0;

  // Color detection overlay (from yellow_detector_node / red_detector_node)
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr color_det_sub_;
  vision_msgs::msg::Detection2DArray::SharedPtr latest_color_dets_;
  std::mutex color_det_mtx_;

  // Pick ROI polygon overlay (from red_detector_node — the world-frame
  // 60×40 cm pick zone projected to pixels). Cached and drawn on every
  // displayed frame so the operator can verify the gate visually.
  // Published with TRANSIENT_LOCAL durability on the publisher side, so
  // we mirror that here to receive the polygon on subscribe.
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr pick_roi_sub_;
  geometry_msgs::msg::PolygonStamped::SharedPtr latest_pick_roi_;
  std::mutex pick_roi_mtx_;

  // Detection accumulator (10 s window)
  std::map<std::string, int> detection_counts_;
  std::mutex                 counts_mtx_;

  // State
  int  frame_count_{0};
  bool display_available_{true};
};

}  // namespace axros_helper_nodes
