// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Implementation of ArmControllerNode. Plans and executes motion to incoming
// PoseStamped targets via MoveIt, with replan throttling and velocity and
// acceleration scaling.

#include "metis_moveit2_perception/arm_controller_node.hpp"
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <thread>

namespace metis_moveit2_perception
{

ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions & options)
: Node("arm_controller", options)
{
  declare_parameter<std::string>("target_pose_topic",       "/target_hover_pose");
  declare_parameter<std::string>("planning_group",          "arm");
  declare_parameter<std::string>("end_effector_link",       "tool_link");
  declare_parameter<double>     ("planning_time_s",         5.0);
  declare_parameter<int>        ("planning_attempts",       10);
  declare_parameter<double>     ("goal_tolerance_m",        0.01);
  declare_parameter<double>     ("min_replan_interval_s",   3.0);
  declare_parameter<double>     ("velocity_scaling",        0.3);
  declare_parameter<double>     ("acceleration_scaling",    0.3);
  declare_parameter<std::string>("robot_description",       "");
  declare_parameter<std::string>("robot_description_semantic", "");

  target_pose_topic_     = get_parameter("target_pose_topic").as_string();
  planning_group_        = get_parameter("planning_group").as_string();
  end_effector_link_     = get_parameter("end_effector_link").as_string();
  planning_time_s_       = get_parameter("planning_time_s").as_double();
  planning_attempts_     = get_parameter("planning_attempts").as_int();
  goal_tolerance_m_      = get_parameter("goal_tolerance_m").as_double();
  min_replan_interval_s_ = get_parameter("min_replan_interval_s").as_double();
  velocity_scaling_      = get_parameter("velocity_scaling").as_double();
  acceleration_scaling_  = get_parameter("acceleration_scaling").as_double();

  RCLCPP_INFO(get_logger(), "ArmControllerNode parameters:");
  RCLCPP_INFO(get_logger(), "  target_pose_topic:     %s", target_pose_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  planning_group:        %s", planning_group_.c_str());
  RCLCPP_INFO(get_logger(), "  end_effector_link:     %s", end_effector_link_.c_str());
  RCLCPP_INFO(get_logger(), "  planning_time_s:       %.1f", planning_time_s_);
  RCLCPP_INFO(get_logger(), "  planning_attempts:     %d", planning_attempts_);
  RCLCPP_INFO(get_logger(), "  goal_tolerance_m:      %.3f", goal_tolerance_m_);
  RCLCPP_INFO(get_logger(), "  min_replan_interval_s: %.1f", min_replan_interval_s_);
  RCLCPP_INFO(get_logger(), "  velocity_scaling:      %.2f", velocity_scaling_);
  RCLCPP_INFO(get_logger(), "  acceleration_scaling:  %.2f", acceleration_scaling_);

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    target_pose_topic_, 10,
    std::bind(&ArmControllerNode::pose_callback, this, std::placeholders::_1));

  last_plan_time_ = this->now() - rclcpp::Duration(100, 0);
}

void ArmControllerNode::initialize()
{
  RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface...");

  // robot_description is passed directly as a parameter from the launch file
  // No SyncParametersClient needed — avoids executor conflict entirely
  auto rd = get_parameter("robot_description").as_string();
  if (rd.empty()) {
    RCLCPP_FATAL(get_logger(),
      "robot_description parameter is empty. "
      "Check that perception_arm.launch.py loaded the URDF correctly.");
    return;
  }
  RCLCPP_INFO(get_logger(),
    "robot_description loaded (%zu bytes)", rd.size());

  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), planning_group_);
  move_group_->setEndEffectorLink(end_effector_link_);
  move_group_->setPlanningTime(planning_time_s_);
  move_group_->setNumPlanningAttempts(planning_attempts_);
  move_group_->setGoalTolerance(goal_tolerance_m_);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
  move_group_ready_ = true;

  RCLCPP_INFO(get_logger(),
    "ArmControllerNode ready — group '%s', end-effector '%s'",
    planning_group_.c_str(), end_effector_link_.c_str());
}

void ArmControllerNode::pose_callback(
  geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!move_group_ready_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "MoveGroupInterface not ready yet -- skipping pose");
    return;
  }

  auto now = this->now();
  if ((now - last_plan_time_).seconds() < min_replan_interval_s_) {
    return;
  }

  // Skip replanning if we're already at (or very near) this target
  if (has_reached_pose_) {
    double dx = msg->pose.position.x - last_reached_pose_.position.x;
    double dy = msg->pose.position.y - last_reached_pose_.position.y;
    double dz = msg->pose.position.z - last_reached_pose_.position.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < goal_tolerance_m_ * 2.0) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
        "Target unchanged (%.3f m from current) — holding position", dist);
      return;
    }
  }

  last_plan_time_ = now;

  RCLCPP_INFO(get_logger(),
    "Planning hover: frame='%s' pos(x=%.3f y=%.3f z=%.3f)",
    msg->header.frame_id.c_str(),
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  move_group_->setPoseTarget(msg->pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const result = move_group_->plan(plan);

  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "Plan succeeded — executing");
    move_group_->execute(plan);
    RCLCPP_INFO(get_logger(), "Motion complete");
    last_reached_pose_ = msg->pose;
    has_reached_pose_ = true;
  } else {
    RCLCPP_WARN(get_logger(), "Planning failed (code: %d)", result.val);
  }
}

}  // namespace metis_moveit2_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<metis_moveit2_perception::ArmControllerNode>(options);

  // MultiThreadedExecutor is required for MoveGroupInterface —
  // it uses internal async service calls that need concurrent spinning
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Initialize MoveGroupInterface from a background thread so that
  // executor.spin() runs on the main thread (clean SIGINT shutdown)
  std::thread init_thread([&node]() {
    // Brief sleep so DDS discovery completes before MoveGroup init
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    node->initialize();
  });
  init_thread.detach();

  executor.spin();
  rclcpp::shutdown();
  return 0;
}