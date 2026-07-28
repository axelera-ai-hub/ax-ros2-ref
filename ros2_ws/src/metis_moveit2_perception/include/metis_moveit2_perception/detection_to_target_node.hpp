// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Declares DetectionToTargetNode, which converts a Detection2DArray into a 3D
// hover PoseStamped in the robot base frame.

#ifndef METIS_MOVEIT2_PERCEPTION__DETECTION_TO_TARGET_NODE_HPP_
#define METIS_MOVEIT2_PERCEPTION__DETECTION_TO_TARGET_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace metis_moveit2_perception
{

class DetectionToTargetNode : public rclcpp::Node
{
public:
  explicit DetectionToTargetNode(const rclcpp::NodeOptions & options);

private:
  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string target_class_;
  double table_height_m_;
  double hover_offset_m_;
  double camera_fx_;
  double camera_fy_;
  double camera_cx_;
  double camera_cy_;
  double assumed_depth_m_;
  double max_reach_m_;
  std::string camera_frame_;
  std::string robot_base_frame_;
};

}  // namespace metis_moveit2_perception

#endif  // METIS_MOVEIT2_PERCEPTION__DETECTION_TO_TARGET_NODE_HPP_
