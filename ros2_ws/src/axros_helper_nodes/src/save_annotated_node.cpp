// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Implementation of SaveAnnotatedNode. Synchronizes images with detections
// and writes annotated frames to disk.

#include "axros_helper_nodes/save_annotated_node.hpp"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace axros_helper_nodes
{

SaveAnnotatedNode::SaveAnnotatedNode(const rclcpp::NodeOptions & options)
: Node("save_annotated", options)
{
  // ── Declare + get parameters 
  declare_parameter<std::string>("image_topic", "image_raw");
  declare_parameter<std::string>("detections_topic", "detections");
  declare_parameter<std::string>("output_dir", "~/experimental.axros/results");
  declare_parameter<std::string>("annotated_topic", "camera_frame_annotated");
  declare_parameter<int>("save_every_n", 1);
  declare_parameter<bool>("save_frames", true);
  declare_parameter<bool>("publish_annotated", true);

  image_topic_       = get_parameter("image_topic").as_string();
  detections_topic_  = get_parameter("detections_topic").as_string();
  output_dir_        = get_parameter("output_dir").as_string();
  annotated_topic_   = get_parameter("annotated_topic").as_string();
  save_every_n_      = get_parameter("save_every_n").as_int();
  save_frames_       = get_parameter("save_frames").as_bool();
  publish_annotated_ = get_parameter("publish_annotated").as_bool();

  // ── Expand in output_dir_ 
  if (!output_dir_.empty() && output_dir_[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home) {
      output_dir_ = std::string(home) + output_dir_.substr(1);
    }
  }

  // ── Create output directory 
  if (save_frames_) {
    std::filesystem::create_directories(output_dir_);
    RCLCPP_INFO(get_logger(), "Saving annotated frames to: %s", output_dir_.c_str());
  }

  // ── Log parameters 
  RCLCPP_INFO(get_logger(),
    "SaveAnnotatedNode:\n"
    "  image_topic:       %s\n"
    "  detections_topic:  %s\n"
    "  annotated_topic:   %s\n"
    "  save_every_n:      %d\n"
    "  save_frames:       %s\n"
    "  publish_annotated: %s",
    image_topic_.c_str(),
    detections_topic_.c_str(),
    annotated_topic_.c_str(),
    save_every_n_,
    save_frames_ ? "true" : "false",
    publish_annotated_ ? "true" : "false");

  // ── message_filters subscribers 
  image_sub_.subscribe(this, image_topic_);
  det_sub_.subscribe(this, detections_topic_);

  // ── ApproximateTimeSynchronizer 
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), image_sub_, det_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
  sync_->registerCallback(&SaveAnnotatedNode::sync_callback, this);

  // ── Annotated publisher 
  if (publish_annotated_) {
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, 10);
  }
}

void SaveAnnotatedNode::sync_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & det_msg)
{
  // ── Convert to cv::Mat 
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat annotated = draw_detections(cv_ptr->image, *det_msg);

  ++frame_count_;

  // ── Save frame 
  if (save_frames_ && (frame_count_ % save_every_n_ == 0)) {
    std::ostringstream ss;
    ss << output_dir_ << "/frame_"
       << std::setfill('0') << std::setw(6) << saved_count_ << ".png";
    cv::imwrite(ss.str(), annotated);
    ++saved_count_;

    if (saved_count_ % 10 == 0) {
      RCLCPP_INFO(get_logger(), "Saved frame %d to %s",
        saved_count_, ss.str().c_str());
    }
  }

  // ── Publish annotated frame 
  if (publish_annotated_) {
    auto out_msg = cv_bridge::CvImage(img_msg->header, "bgr8", annotated).toImageMsg();
    annotated_pub_->publish(*out_msg);
  }
}

cv::Mat SaveAnnotatedNode::draw_detections(
  const cv::Mat & frame,
  const vision_msgs::msg::Detection2DArray & detections)
{
  cv::Mat out = frame.clone();

  for (const auto & det : detections.detections) {
    if (det.results.empty()) {
      continue;
    }

    const auto & hyp = det.results[0].hypothesis;
    float score = static_cast<float>(hyp.score);
    std::string label = hyp.class_id;

    cv::Scalar color;
    if (score >= 0.7f) {
      color = cv::Scalar(0, 255, 0);    // green — high confidence
    } else if (score >= 0.4f) {
      color = cv::Scalar(0, 165, 255);  // orange — medium confidence
    } else {
      color = cv::Scalar(0, 0, 255);    // red — low confidence
    }

    // Bounding box corners from center + size
    int x1 = static_cast<int>(det.bbox.center.position.x - det.bbox.size_x / 2.0);
    int y1 = static_cast<int>(det.bbox.center.position.y - det.bbox.size_y / 2.0);
    int x2 = static_cast<int>(det.bbox.center.position.x + det.bbox.size_x / 2.0);
    int y2 = static_cast<int>(det.bbox.center.position.y + det.bbox.size_y / 2.0);

    // Clamp to frame bounds
    x1 = std::clamp(x1, 0, out.cols - 1);
    y1 = std::clamp(y1, 0, out.rows - 1);
    x2 = std::clamp(x2, 0, out.cols - 1);
    y2 = std::clamp(y2, 0, out.rows - 1);

    cv::rectangle(out, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);

    // Label text
    std::ostringstream label_ss;
    label_ss << label << " " << std::fixed << std::setprecision(2) << score;
    std::string label_text = label_ss.str();

    int baseline = 0;
    cv::Size text_size = cv::getTextSize(
      label_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

    // Filled rectangle behind text
    cv::rectangle(out,
      cv::Point(x1, y1 - text_size.height - 6),
      cv::Point(x1 + text_size.width, y1),
      color, cv::FILLED);

    // Text in black over colored background
    cv::putText(out, label_text,
      cv::Point(x1, y1 - 4),
      cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

    // Center dot
    int cx = (x1 + x2) / 2;
    int cy = (y1 + y2) / 2;
    cv::circle(out, cv::Point(cx, cy), 3, color, cv::FILLED);
  }

  // Detection count overlay
  std::string count_text = "Detections: " +
    std::to_string(detections.detections.size());
  cv::putText(out, count_text, cv::Point(10, 30),
    cv::FONT_HERSHEY_SIMPLEX, 0.8,
    cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

  return out;
}

}  // namespace axros_helper_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(axros_helper_nodes::SaveAnnotatedNode)
