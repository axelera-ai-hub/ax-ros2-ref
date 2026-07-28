// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Declares ArmControllerNode, which subscribes to target hover poses and
// drives the NED2 arm to them via the MoveIt MoveGroupInterface.

#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

namespace metis_moveit2_perception
{

class ArmControllerNode : public rclcpp::Node
{
public:
  explicit ArmControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Called AFTER construction — initializes MoveGroupInterface.
  // Must be called from outside after the node is added to executor.
  void initialize();

private:
  void pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Parameters
  std::string target_pose_topic_;
  std::string planning_group_;
  std::string end_effector_link_;
  double      planning_time_s_;
  int         planning_attempts_;
  double      goal_tolerance_m_;
  double      min_replan_interval_s_;
  double      velocity_scaling_;
  double      acceleration_scaling_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>   move_group_;
  rclcpp::Time                                                       last_plan_time_;
  bool                                                               move_group_ready_{false};
  geometry_msgs::msg::Pose                                           last_reached_pose_;
  bool                                                               has_reached_pose_{false};
};

}  // namespace metis_moveit2_perception
