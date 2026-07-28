// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Implementation of DetectionToTargetNode. Back-projects a 2D detection to a
// 3D hover pose using a pinhole model and a table-height assumption.

#include "metis_moveit2_perception/detection_to_target_node.hpp"

#include <cmath>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace metis_moveit2_perception
{

DetectionToTargetNode::DetectionToTargetNode(const rclcpp::NodeOptions & options)
: Node("detection_to_target", options)
{
  // Declare and get parameters
  auto detections_topic = this->declare_parameter<std::string>("detections_topic", "/detections");
  target_class_ = this->declare_parameter<std::string>("target_class", "cat");
  table_height_m_ = this->declare_parameter<double>("table_height_m", 0.0);
  hover_offset_m_ = this->declare_parameter<double>("hover_offset_m", 0.15);
  camera_fx_ = this->declare_parameter<double>("camera_fx", 1360.0);
  camera_fy_ = this->declare_parameter<double>("camera_fy", 1360.0);
  camera_cx_ = this->declare_parameter<double>("camera_cx", 960.0);
  camera_cy_ = this->declare_parameter<double>("camera_cy", 540.0);
  assumed_depth_m_ = this->declare_parameter<double>("assumed_depth_m", 0.5);
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera");
  robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame", "world");
  max_reach_m_ = this->declare_parameter<double>("max_reach_m", 0.38);
  auto target_pose_topic = this->declare_parameter<std::string>("target_pose_topic",
      "/target_hover_pose");

  RCLCPP_INFO(get_logger(), "DetectionToTargetNode parameters:");
  RCLCPP_INFO(get_logger(), "  detections_topic:  %s", detections_topic.c_str());
  RCLCPP_INFO(get_logger(), "  target_class:      %s", target_class_.c_str());
  RCLCPP_INFO(get_logger(), "  table_height_m:    %.3f", table_height_m_);
  RCLCPP_INFO(get_logger(), "  hover_offset_m:    %.3f", hover_offset_m_);
  RCLCPP_INFO(get_logger(), "  camera_fx:         %.1f", camera_fx_);
  RCLCPP_INFO(get_logger(), "  camera_fy:         %.1f", camera_fy_);
  RCLCPP_INFO(get_logger(), "  camera_cx:         %.1f", camera_cx_);
  RCLCPP_INFO(get_logger(), "  camera_cy:         %.1f", camera_cy_);
  RCLCPP_INFO(get_logger(), "  assumed_depth_m:   %.3f", assumed_depth_m_);
  RCLCPP_INFO(get_logger(), "  camera_frame:      %s", camera_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  robot_base_frame:  %s", robot_base_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  max_reach_m:       %.3f", max_reach_m_);
  RCLCPP_INFO(get_logger(), "  target_pose_topic: %s", target_pose_topic.c_str());

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscription and publisher
  det_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    detections_topic, 10,
    std::bind(&DetectionToTargetNode::detection_callback, this, std::placeholders::_1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    target_pose_topic, 10);
}

void DetectionToTargetNode::detection_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  // Find highest-confidence detection matching target_class_
  const vision_msgs::msg::Detection2D * best = nullptr;
  double best_score = -1.0;

  for (const auto & det : msg->detections) {
    if (det.results.empty()) {continue;}
    const auto & hyp = det.results[0].hypothesis;
    if (hyp.class_id == target_class_ && hyp.score > best_score) {
      best_score = hyp.score;
      best = &det;
    }
  }

  if (!best) {return;}

  // Extract bounding box center
  double u = best->bbox.center.position.x;
  double v = best->bbox.center.position.y;

  // Project to 3D in camera frame using pinhole model
  double X_cam = (u - camera_cx_) * assumed_depth_m_ / camera_fx_;
  double Y_cam = (v - camera_cy_) * assumed_depth_m_ / camera_fy_;
  double Z_cam = assumed_depth_m_;

  // Build PointStamped in camera frame
  geometry_msgs::msg::PointStamped point_cam;
  point_cam.header.stamp = msg->header.stamp;
  point_cam.header.frame_id = camera_frame_;
  point_cam.point.x = X_cam;
  point_cam.point.y = Y_cam;
  point_cam.point.z = Z_cam;

  // Transform to robot base frame
  geometry_msgs::msg::PointStamped point_base;
  try {
    point_base = tf_buffer_->transform(point_cam, robot_base_frame_);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
    return;
  }

  // Clamp XY distance to robot workspace radius, preserving direction
  double dx = point_base.point.x;
  double dy = point_base.point.y;
  double dist = std::hypot(dx, dy);
  if (dist > max_reach_m_ && dist > 0.0) {
    double scale = max_reach_m_ / dist;
    dx *= scale;
    dy *= scale;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Target at %.3f m clamped to %.3f m (workspace limit)",
      dist, max_reach_m_);
  }

  // Build hover PoseStamped in robot base frame
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = robot_base_frame_;
  pose.pose.position.x = dx;
  pose.pose.position.y = dy;
  pose.pose.position.z = table_height_m_ + hover_offset_m_;
  // Quaternion for tool_link pointing straight down in world frame
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.707;

  pose_pub_->publish(pose);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "Target '%s' -> hover pose world(x=%.3f y=%.3f z=%.3f)",
    target_class_.c_str(),
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z);
}

}  // namespace metis_moveit2_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(metis_moveit2_perception::DetectionToTargetNode)
