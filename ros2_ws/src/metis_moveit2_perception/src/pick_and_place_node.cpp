// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Implementation of PickAndPlaceNode. Full pick and place state machine:
// detection, approach, Cartesian descent, grip verification, and placement
// on the NED2 arm.

#include "metis_moveit2_perception/pick_and_place_node.hpp"

#include <algorithm>
#include <limits>
#include <thread>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

namespace metis_moveit2_perception
{

static const char * state_name(PnPState s)
{
  switch (s) {
    case PnPState::WAITING_FOR_INIT:      return "WAITING_FOR_INIT";
    case PnPState::WAITING_FOR_HAND:      return "WAITING_FOR_HAND";
    case PnPState::DETECTING_PICK:        return "DETECTING_PICK";
    case PnPState::APPROACHING_PICK:      return "APPROACHING_PICK";
    case PnPState::OPENING_GRIPPER:       return "OPENING_GRIPPER";
    case PnPState::DESCENDING_TO_PICK:    return "DESCENDING_TO_PICK";
    case PnPState::GRIPPING:              return "GRIPPING";
    case PnPState::LIFTING:               return "LIFTING";
    case PnPState::VERIFYING_GRIP:        return "VERIFYING_GRIP";
    case PnPState::DETECTING_PLACE:       return "DETECTING_PLACE";
    case PnPState::ADAPTIVE_PLACE_HOVER:  return "ADAPTIVE_PLACE_HOVER";
    case PnPState::APPROACHING_PLACE:     return "APPROACHING_PLACE";
    case PnPState::DESCENDING_TO_PLACE:   return "DESCENDING_TO_PLACE";
    case PnPState::RELEASING:             return "RELEASING";
    case PnPState::RETRACTING:            return "RETRACTING";
    case PnPState::LOOP_CHECKING_SCENE:   return "LOOP_CHECKING_SCENE";
    case PnPState::DONE:                  return "DONE";
  }
  return "UNKNOWN";
}

PickAndPlaceNode::PickAndPlaceNode(const rclcpp::NodeOptions & options)
: Node("pick_and_place", options)
{
  // Declare parameters
  declare_parameter<std::string>("pick_class",         "duck");
  declare_parameter<std::string>("place_class",        "cow");
  declare_parameter<double>     ("camera_fx",          1360.0);
  declare_parameter<double>     ("camera_fy",          1360.0);
  declare_parameter<double>     ("camera_cx",          960.0);
  declare_parameter<double>     ("camera_cy",          540.0);
  declare_parameter<double>     ("assumed_depth_m",    0.42);
  // Separate depth for place projection (hand above the table is closer
  // to the camera than the table itself). -1.0 sentinel = "use same as
  // assumed_depth_m" — keeps every existing launch backwards-compatible.
  declare_parameter<double>     ("place_assumed_depth_m", -1.0);
  // Image size used by the place-candidate edge filter. Defaults match
  // the Pi C920 capture at 1920x1080; override via launch when needed.
  declare_parameter<int>        ("image_width",          1920);
  declare_parameter<int>        ("image_height",         1080);
  // Reject place candidates whose bbox centre lies within this many
  // pixels of the image border. 0 disables the gate (default behaviour).
  declare_parameter<int>        ("place_edge_margin_px",    0);
  // Place dwell-lock — only consulted when place_dwell_time_s > 0. The
  // most-central candidate must stay within tolerance for the dwell
  // duration before becoming the latched place target.
  declare_parameter<double>     ("place_dwell_time_s",      0.0);
  declare_parameter<double>     ("place_dwell_tolerance_m", 0.05);
  // Sentinel (-1) → use max_reach_m. Tighter than max_reach for hand mode
  // because NED2's reach shrinks fast at high Z.
  declare_parameter<double>     ("place_max_reach_m",      -1.0);
  declare_parameter<std::string>("camera_frame",       "camera");
  declare_parameter<std::string>("robot_base_frame",   "world");
  declare_parameter<double>     ("max_reach_m",        0.38);
  // Pick-side world-frame ROI. Disabled by default — the kitkat HSV path
  // already gates upstream in red_detector_node, so only the YOLO-kitkat
  // launch path turns this on.
  declare_parameter<bool>       ("pick_roi_enabled",      false);
  declare_parameter<double>     ("pick_roi_x_min_m",     -1.0);
  declare_parameter<double>     ("pick_roi_x_max_m",      1.0);
  declare_parameter<double>     ("pick_roi_y_min_m",     -1.0);
  declare_parameter<double>     ("pick_roi_y_max_m",      1.0);
  // Pre-pick hand gate (default off). When true, every pick cycle waits
  // in WAITING_FOR_HAND until a place_class detection has been visible
  // continuously for hand_gate_duration_s_ before starting to look for
  // the pick object. Used to make the robot operator-triggered: it does
  // nothing until you show your palm, then it goes and fetches a bar.
  declare_parameter<bool>       ("wait_for_hand_before_pick", false);
  declare_parameter<double>     ("hand_gate_duration_s",    1.0);
  declare_parameter<double>     ("hand_gate_stale_s",       0.3);
  // Adaptive place mode — the arm hover-tracks the hand and only commits
  // the descent once the existing dwell-lock satisfies AND the arm has
  // caught up. Disabled by default; opt-in via ADAPTIVE_PLACE=1.
  declare_parameter<bool>       ("adaptive_place_enabled",        false);
  declare_parameter<double>     ("adaptive_follow_threshold_m",   0.04);
  declare_parameter<double>     ("adaptive_arm_close_m",          0.06);
  declare_parameter<double>     ("hover_height_m",      0.12);
  // Separate hover for the pick side. Sentinel (-1) means "use hover_height_m".
  // The hand-place profile sets hover_height_m for the *place* approach
  // (0.37 m above an operator's palm) — that's too high for picking off
  // the table, so this lets the pick hover stay at the table-friendly value.
  declare_parameter<double>     ("pick_hover_height_m",   -1.0);
  declare_parameter<double>     ("grasp_height_m",      0.04);
  declare_parameter<double>     ("place_drop_height_m", 0.12);
  declare_parameter<double>     ("lift_height_m",       0.18);
  declare_parameter<double>     ("pick_offset_x",          0.0);
  declare_parameter<double>     ("pick_offset_y",          0.0);
  declare_parameter<double>     ("min_detection_score",    0.45);
  declare_parameter<double>     ("pick_ema_alpha",         0.25);
  declare_parameter<int>        ("tool_id",            11);
  declare_parameter<int>        ("max_torque_pct",     100);
  declare_parameter<int>        ("hold_torque_pct",    50);
  declare_parameter<double>     ("grasp_min_height_m", 0.09);
  declare_parameter<double>     ("verify_duration_s",  3.0);
  declare_parameter<double>     ("verify_radius_m",    0.15);
  declare_parameter<bool>       ("retry_on_failed_verify", false);
  declare_parameter<int>        ("max_grip_attempts",  1);
  // Number of times APPROACHING_PICK retries the same latched pose before
  // giving up, dropping the latch, and going back to DETECTING_PICK. Caps
  // the wall-clock spent thrashing on an unreachable candidate (each plan
  // attempt is ~planning_time_s seconds). Set to 1 to disable retries.
  declare_parameter<int>        ("max_approach_pick_attempts", 3);
  declare_parameter<bool>       ("use_fixed_drop_pose", false);
  declare_parameter<double>     ("fixed_drop_x",       0.20);
  declare_parameter<double>     ("fixed_drop_y",       0.20);
  // Plan B fallback — skip kitkat detection on the pick side and grasp
  // at a fixed XY. Drop-side detection (hand-place) keeps running as-is.
  declare_parameter<bool>       ("use_fixed_pick_pose", false);
  declare_parameter<double>     ("fixed_pick_x",       0.30);
  declare_parameter<double>     ("fixed_pick_y",       0.0);
  declare_parameter<int>        ("max_fixed_pick_attempts", 1);
  declare_parameter<std::string>("place_mode",         "detection");
  declare_parameter<std::string>("place_tf_frame",     "");
  declare_parameter<double>     ("place_tf_max_age_s", 1.0);
  declare_parameter<double>     ("place_tf_hover_offset_m", 0.05);
  declare_parameter<double>     ("place_tf_drop_offset_m",  0.00);
  declare_parameter<double>     ("place_tf_lift_offset_m",  0.10);
  declare_parameter<double>     ("place_offset_x_m",        0.00);
  declare_parameter<double>     ("place_offset_y_m",        0.00);
  declare_parameter<bool>       ("loop_mode",          false);
  declare_parameter<double>     ("loop_scene_timeout_s", 5.0);
  declare_parameter<std::string>("pick_selection_mode", "nearest_to_base");
  declare_parameter<double>     ("pick_accept_radius_m", 0.03);
  declare_parameter<double>     ("drop_stack_increment_m", 0.0);
  declare_parameter<int>        ("min_pick_detections", 10);
  declare_parameter<std::string>("planning_group",     "arm");
  declare_parameter<std::string>("end_effector_link",  "tool_link");
  declare_parameter<double>     ("planning_time_s",    5.0);
  declare_parameter<int>        ("planning_attempts",  10);
  declare_parameter<double>     ("velocity_scaling",   0.15);
  declare_parameter<double>     ("acceleration_scaling", 0.15);
  declare_parameter<std::string>("robot_description",  "");
  declare_parameter<std::string>("robot_description_semantic", "");

  // Read parameters
  pick_class_       = get_parameter("pick_class").as_string();
  place_class_      = get_parameter("place_class").as_string();
  // Parse comma-separated place_class into a list (e.g. "palm,fist").
  // Wildcard "*" leaves the list empty and is handled separately. Empty
  // entries / whitespace are stripped so "palm, fist" works too.
  place_class_set_.clear();
  if (place_class_ != "*") {
    std::string cur;
    for (char c : place_class_) {
      if (c == ',') {
        if (!cur.empty()) place_class_set_.push_back(cur);
        cur.clear();
      } else if (c != ' ' && c != '\t') {
        cur.push_back(c);
      }
    }
    if (!cur.empty()) place_class_set_.push_back(cur);
  }
  camera_fx_        = get_parameter("camera_fx").as_double();
  camera_fy_        = get_parameter("camera_fy").as_double();
  camera_cx_        = get_parameter("camera_cx").as_double();
  camera_cy_        = get_parameter("camera_cy").as_double();
  assumed_depth_m_  = get_parameter("assumed_depth_m").as_double();
  place_assumed_depth_m_ = get_parameter("place_assumed_depth_m").as_double();
  if (place_assumed_depth_m_ <= 0.0) {
    place_assumed_depth_m_ = assumed_depth_m_;
  }
  image_width_         = get_parameter("image_width").as_int();
  image_height_        = get_parameter("image_height").as_int();
  place_edge_margin_px_= get_parameter("place_edge_margin_px").as_int();
  place_dwell_time_s_      = get_parameter("place_dwell_time_s").as_double();
  place_dwell_tolerance_m_ = get_parameter("place_dwell_tolerance_m").as_double();
  place_max_reach_m_       = get_parameter("place_max_reach_m").as_double();
  if (place_max_reach_m_ <= 0.0) place_max_reach_m_ = max_reach_m_;
  camera_frame_     = get_parameter("camera_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  max_reach_m_      = get_parameter("max_reach_m").as_double();
  pick_roi_enabled_   = get_parameter("pick_roi_enabled").as_bool();
  pick_roi_x_min_m_   = get_parameter("pick_roi_x_min_m").as_double();
  pick_roi_x_max_m_   = get_parameter("pick_roi_x_max_m").as_double();
  pick_roi_y_min_m_   = get_parameter("pick_roi_y_min_m").as_double();
  pick_roi_y_max_m_   = get_parameter("pick_roi_y_max_m").as_double();
  wait_for_hand_before_pick_ =
    get_parameter("wait_for_hand_before_pick").as_bool();
  hand_gate_duration_s_      = get_parameter("hand_gate_duration_s").as_double();
  hand_gate_stale_s_         = get_parameter("hand_gate_stale_s").as_double();
  adaptive_place_enabled_      = get_parameter("adaptive_place_enabled").as_bool();
  adaptive_follow_threshold_m_ = get_parameter("adaptive_follow_threshold_m").as_double();
  adaptive_arm_close_m_        = get_parameter("adaptive_arm_close_m").as_double();
  hover_height_m_      = get_parameter("hover_height_m").as_double();
  pick_hover_height_m_ = get_parameter("pick_hover_height_m").as_double();
  if (pick_hover_height_m_ <= 0.0) pick_hover_height_m_ = hover_height_m_;
  grasp_height_m_      = get_parameter("grasp_height_m").as_double();
  place_drop_height_m_ = get_parameter("place_drop_height_m").as_double();
  lift_height_m_       = get_parameter("lift_height_m").as_double();
  pick_offset_x_         = get_parameter("pick_offset_x").as_double();
  pick_offset_y_         = get_parameter("pick_offset_y").as_double();
  min_detection_score_   = get_parameter("min_detection_score").as_double();
  pick_ema_alpha_        = get_parameter("pick_ema_alpha").as_double();
  tool_id_          = get_parameter("tool_id").as_int();
  max_torque_pct_   = static_cast<uint8_t>(get_parameter("max_torque_pct").as_int());
  hold_torque_pct_  = static_cast<uint8_t>(get_parameter("hold_torque_pct").as_int());
  grasp_min_height_m_  = get_parameter("grasp_min_height_m").as_double();
  verify_duration_s_   = get_parameter("verify_duration_s").as_double();
  verify_radius_m_     = get_parameter("verify_radius_m").as_double();
  retry_on_failed_verify_ = get_parameter("retry_on_failed_verify").as_bool();
  max_grip_attempts_   = get_parameter("max_grip_attempts").as_int();
  max_approach_pick_attempts_ = get_parameter("max_approach_pick_attempts").as_int();
  use_fixed_drop_pose_ = get_parameter("use_fixed_drop_pose").as_bool();
  fixed_drop_x_        = get_parameter("fixed_drop_x").as_double();
  fixed_drop_y_        = get_parameter("fixed_drop_y").as_double();
  use_fixed_pick_pose_     = get_parameter("use_fixed_pick_pose").as_bool();
  fixed_pick_x_            = get_parameter("fixed_pick_x").as_double();
  fixed_pick_y_            = get_parameter("fixed_pick_y").as_double();
  max_fixed_pick_attempts_ = get_parameter("max_fixed_pick_attempts").as_int();
  place_mode_          = get_parameter("place_mode").as_string();
  place_tf_frame_      = get_parameter("place_tf_frame").as_string();
  place_tf_max_age_s_  = get_parameter("place_tf_max_age_s").as_double();
  place_tf_hover_offset_m_ = get_parameter("place_tf_hover_offset_m").as_double();
  place_tf_drop_offset_m_  = get_parameter("place_tf_drop_offset_m").as_double();
  place_tf_lift_offset_m_  = get_parameter("place_tf_lift_offset_m").as_double();
  place_offset_x_m_        = get_parameter("place_offset_x_m").as_double();
  place_offset_y_m_        = get_parameter("place_offset_y_m").as_double();

  // Back-compat: the old boolean use_fixed_drop_pose still wins if set. This
  // lets existing launches keep working untouched.
  if (use_fixed_drop_pose_ && place_mode_ == "detection") {
    place_mode_ = "fixed";
  }
  if (place_mode_ != "detection" && place_mode_ != "fixed" && place_mode_ != "tf") {
    RCLCPP_WARN(get_logger(),
      "Unknown place_mode '%s' — falling back to 'detection'",
      place_mode_.c_str());
    place_mode_ = "detection";
  }
  if (place_mode_ == "tf" && place_tf_frame_.empty()) {
    RCLCPP_WARN(get_logger(),
      "place_mode='tf' but place_tf_frame is empty — falling back to 'detection'");
    place_mode_ = "detection";
  }
  loop_mode_           = get_parameter("loop_mode").as_bool();
  loop_scene_timeout_s_ = get_parameter("loop_scene_timeout_s").as_double();
  pick_selection_mode_ = get_parameter("pick_selection_mode").as_string();
  pick_accept_radius_m_ = get_parameter("pick_accept_radius_m").as_double();
  drop_stack_increment_m_ = get_parameter("drop_stack_increment_m").as_double();
  min_pick_detections_ = get_parameter("min_pick_detections").as_int();
  if (min_pick_detections_ < 1) min_pick_detections_ = 1;

  if (pick_selection_mode_ != "nearest_to_base" &&
      pick_selection_mode_ != "rightmost" &&
      pick_selection_mode_ != "leftmost" &&
      pick_selection_mode_ != "nearest_to_x_axis") {
    RCLCPP_WARN(get_logger(),
      "Unknown pick_selection_mode '%s' — falling back to 'nearest_to_base'",
      pick_selection_mode_.c_str());
    pick_selection_mode_ = "nearest_to_base";
  }

  RCLCPP_INFO(get_logger(), "PickAndPlaceNode parameters:");
  RCLCPP_INFO(get_logger(), "  pick_class:     %s", pick_class_.c_str());
  RCLCPP_INFO(get_logger(), "  place_class:    %s", place_class_.c_str());
  RCLCPP_INFO(get_logger(), "  hover_height_m:      %.3f (pick uses %.3f)",
    hover_height_m_, pick_hover_height_m_);
  RCLCPP_INFO(get_logger(), "  grasp_height_m:      %.3f", grasp_height_m_);
  RCLCPP_INFO(get_logger(), "  grasp_min_height_m:  %.3f", grasp_min_height_m_);
  RCLCPP_INFO(get_logger(), "  place_drop_height_m: %.3f", place_drop_height_m_);
  RCLCPP_INFO(get_logger(), "  lift_height_m:       %.3f", lift_height_m_);
  RCLCPP_INFO(get_logger(), "  verify_duration_s:   %.2f", verify_duration_s_);
  RCLCPP_INFO(get_logger(), "  verify_radius_m:     %.3f", verify_radius_m_);
  RCLCPP_INFO(get_logger(), "  retry_on_failed_verify: %s",
    retry_on_failed_verify_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  max_grip_attempts:   %d", max_grip_attempts_);
  RCLCPP_INFO(get_logger(), "  use_fixed_pick_pose: %s", use_fixed_pick_pose_ ? "true" : "false");
  if (use_fixed_pick_pose_) {
    RCLCPP_INFO(get_logger(), "    fixed_pick: (%.3f, %.3f)", fixed_pick_x_, fixed_pick_y_);
    RCLCPP_INFO(get_logger(), "    max_fixed_pick_attempts: %d", max_fixed_pick_attempts_);
  }
  RCLCPP_INFO(get_logger(), "  use_fixed_drop_pose: %s", use_fixed_drop_pose_ ? "true" : "false");
  if (use_fixed_drop_pose_) {
    RCLCPP_INFO(get_logger(), "    fixed_drop: (%.3f, %.3f)", fixed_drop_x_, fixed_drop_y_);
  }
  RCLCPP_INFO(get_logger(), "  place_mode:     %s", place_mode_.c_str());
  RCLCPP_INFO(get_logger(),
    "  place XY bias (m): (%.3f, %.3f)",
    place_offset_x_m_, place_offset_y_m_);
  if (place_mode_ == "tf") {
    RCLCPP_INFO(get_logger(), "    place_tf_frame: %s", place_tf_frame_.c_str());
    RCLCPP_INFO(get_logger(), "    place_tf_max_age_s: %.2f", place_tf_max_age_s_);
    RCLCPP_INFO(get_logger(),
      "    hover/drop/lift offsets (m): %.2f / %.2f / %.2f",
      place_tf_hover_offset_m_, place_tf_drop_offset_m_, place_tf_lift_offset_m_);
  }
  if (place_mode_ == "detection") {
    RCLCPP_INFO(get_logger(),
      "    place_assumed_depth_m: %.3f (pick uses %.3f)",
      place_assumed_depth_m_, assumed_depth_m_);
    {
      std::string class_summary;
      if (place_class_ == "*") {
        class_summary = "wildcard (any class)";
      } else if (place_class_set_.size() <= 1) {
        class_summary = "class '" + place_class_ + "'";
      } else {
        class_summary = "classes {";
        for (size_t i = 0; i < place_class_set_.size(); ++i) {
          if (i) class_summary += ", ";
          class_summary += place_class_set_[i];
        }
        class_summary += "}";
      }
      RCLCPP_INFO(get_logger(),
        "    image %dx%d, place_edge_margin_px: %d, match: %s",
        image_width_, image_height_, place_edge_margin_px_,
        class_summary.c_str());
    }
    if (place_dwell_time_s_ > 0.0) {
      RCLCPP_INFO(get_logger(),
        "    dwell-lock: hold within %.2f m for %.2f s",
        place_dwell_tolerance_m_, place_dwell_time_s_);
    } else {
      RCLCPP_INFO(get_logger(), "    dwell-lock: disabled (immediate latch)");
    }
  }
  RCLCPP_INFO(get_logger(), "  loop_mode:           %s", loop_mode_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  pick_selection_mode: %s", pick_selection_mode_.c_str());
  RCLCPP_INFO(get_logger(), "  pick_accept_radius_m: %.3f", pick_accept_radius_m_);
  RCLCPP_INFO(get_logger(), "  drop_stack_increment_m: %.4f", drop_stack_increment_m_);
  RCLCPP_INFO(get_logger(), "  min_pick_detections: %d", min_pick_detections_);
  RCLCPP_INFO(get_logger(), "  tool_id:        %d", tool_id_);

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Detection subscriber
  det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detections", 10,
    std::bind(&PickAndPlaceNode::detection_callback, this, std::placeholders::_1));

  // Gripper action client
  gripper_client_ = rclcpp_action::create_client<ToolAction>(
    this, "/niryo_robot_tools_commander/action_server");

  // State machine timer — ticks every 500ms
  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PickAndPlaceNode::tick, this));
}

void PickAndPlaceNode::initialize()
{
  RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface...");

  auto rd = get_parameter("robot_description").as_string();
  if (rd.empty()) {
    RCLCPP_FATAL(get_logger(), "robot_description parameter is empty.");
    return;
  }
  RCLCPP_INFO(get_logger(), "robot_description loaded (%zu bytes)", rd.size());

  auto group = get_parameter("planning_group").as_string();
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), group);
  move_group_->setEndEffectorLink(get_parameter("end_effector_link").as_string());
  move_group_->setPlanningTime(get_parameter("planning_time_s").as_double());
  move_group_->setNumPlanningAttempts(get_parameter("planning_attempts").as_int());
  move_group_->setGoalTolerance(0.02);
  move_group_->setMaxVelocityScalingFactor(get_parameter("velocity_scaling").as_double());
  move_group_->setMaxAccelerationScalingFactor(get_parameter("acceleration_scaling").as_double());
  move_group_ready_ = true;
  state_ = wait_for_hand_before_pick_ ? PnPState::WAITING_FOR_HAND
                                       : PnPState::DETECTING_PICK;

  if (wait_for_hand_before_pick_) {
    RCLCPP_INFO(get_logger(),
      "PickAndPlaceNode ready — hand-gate ON, waiting for '%s' "
      "(hold %.1fs to trigger pick of '%s')",
      place_class_.c_str(), hand_gate_duration_s_, pick_class_.c_str());
  } else {
    RCLCPP_INFO(get_logger(),
      "PickAndPlaceNode ready — waiting for '%s' detection",
      pick_class_.c_str());
  }
}

// ── Detection callback ───────────────────────────────────────────────────────

void PickAndPlaceNode::detection_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  // Collect the pick-class detections from this frame into world coordinates
  // first, then commit to at most one per frame. Without this step, several
  // objects in the same message get EMA-blended together and the target
  // lands on the centroid — i.e. empty table between them.
  std::vector<geometry_msgs::msg::PointStamped> pick_candidates;
  pick_candidates.reserve(msg->detections.size());

  // Place candidates carry pixel (u,v) too so we can sort by image-centre
  // distance after the loop and pick the most central hand. This matters in
  // wildcard mode (place_class_=="*") where several gestures may fire at
  // once — without centre priority a reflection near the frame edge can
  // win the latch.
  struct PlaceCandidate {
    double u;
    double v;
    geometry_msgs::msg::PointStamped world_pt;
  };
  std::vector<PlaceCandidate> place_candidates;
  place_candidates.reserve(msg->detections.size());

  for (const auto & det : msg->detections) {
    if (det.results.empty()) continue;
    const auto & hyp = det.results[0].hypothesis;
    if (hyp.score < min_detection_score_) continue;

    double u = det.bbox.center.position.x;
    double v = det.bbox.center.position.y;

    // -- Pick branch (pinhole back-projection at table depth) --
    if (hyp.class_id == pick_class_ && !pick_target_latched_) {
      geometry_msgs::msg::PointStamped world_pt;
      if (!project_to_world(u, v, assumed_depth_m_, world_pt)) continue;
      // World-frame ROI gate. Mirrors the in-image mask the HSV detector
      // applies in red_detector_node — used in the YOLO-kitkat path where
      // detections come from a class-specific model but can still land
      // anywhere in the camera FOV. The gate uses the raw back-projection
      // (no pick_offset_*) because the ROI describes the physical workspace
      // patch where bars are allowed to be, not where the gripper finally
      // lands. Place / gesture detections are NOT gated.
      if (pick_roi_enabled_) {
        const double wx = world_pt.point.x;
        const double wy = world_pt.point.y;
        if (wx < pick_roi_x_min_m_ || wx > pick_roi_x_max_m_ ||
            wy < pick_roi_y_min_m_ || wy > pick_roi_y_max_m_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
            "pick: rejecting candidate at world (%.3f, %.3f) — outside ROI "
            "X[%.3f,%.3f] Y[%.3f,%.3f]",
            wx, wy,
            pick_roi_x_min_m_, pick_roi_x_max_m_,
            pick_roi_y_min_m_, pick_roi_y_max_m_);
          continue;
        }
      }
      // Reject candidates whose post-offset approach pose falls outside
      // the reach envelope. The arm doesn't move to the raw detection —
      // it moves to (detection + pick_offset), so that's the radius that
      // actually has to be reachable. Gating on the raw detection would
      // both let unreachable approaches through (when the offset pulls
      // *outward*) and reject reachable ones (when it pulls inward).
      // Without this gate make_down_pose() would silently clamp the XY
      // to the edge of the workspace circle and the gripper would close
      // on empty air — the bar that's actually further out gets
      // re-detected post-lift and we loop forever on "grip FAILED".
      const double approach_x = world_pt.point.x + pick_offset_x_;
      const double approach_y = world_pt.point.y + pick_offset_y_;
      const double pick_radius = std::hypot(approach_x, approach_y);
      if (pick_radius > max_reach_m_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "pick: skipping candidate at (%.3f, %.3f) — approach (%.3f, %.3f) "
          "radius %.2fm > max_reach %.2fm (move it closer or raise max_reach_m)",
          world_pt.point.x, world_pt.point.y, approach_x, approach_y,
          pick_radius, max_reach_m_);
        continue;
      }
      // During grip verification the arm holds the object near the clear pose.
      // Only count detections near the original pick spot — far-away ones are
      // other objects that belong to the NEXT cycle, not this one.
      if (verify_moved_clear_) {
        double dx = world_pt.point.x - pre_verify_pick_target_.point.x;
        double dy = world_pt.point.y - pre_verify_pick_target_.point.y;
        if (std::hypot(dx, dy) > verify_radius_m_) continue;
      }
      pick_candidates.push_back(world_pt);
    }

    // -- Place branch (wildcard / multi-class, edge gate, hand-depth) --
    if (!place_target_latched_ && place_mode_ == "detection") {
      bool place_match;
      if (place_class_ == "*") {
        place_match = true;
      } else if (place_class_set_.size() == 1) {
        // Single-class fast path (preserves the original behaviour for
        // simple PLACE_CLASS=palm-style invocations).
        place_match = (hyp.class_id == place_class_set_[0]);
      } else {
        place_match = std::find(place_class_set_.begin(),
                                place_class_set_.end(),
                                hyp.class_id) != place_class_set_.end();
      }
      if (place_match) {
        // Edge gate: discard detections within margin_px of any image border.
        // 0 disables the gate (existing single-class behaviour).
        if (place_edge_margin_px_ > 0) {
          const int margin = place_edge_margin_px_;
          if (u < margin || u > image_width_  - margin ||
              v < margin || v > image_height_ - margin) {
            continue;
          }
        }
        geometry_msgs::msg::PointStamped world_pt;
        if (!project_to_world(u, v, place_assumed_depth_m_, world_pt)) {
          continue;
        }
        // Reach gate: at high place-hover Z the reachable horizontal
        // envelope is much smaller than at the table, so we apply a
        // tighter radius limit specifically for place candidates.
        // Anything beyond is silently dropped — the dwell never forms,
        // pick_and_place holds the bar at hover and waits.
        const double place_radius =
          std::hypot(world_pt.point.x, world_pt.point.y);
        if (place_radius > place_max_reach_m_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
            "place: skipping candidate at (%.3f, %.3f) — radius %.2fm > "
            "place_max_reach %.2fm (hand too far out)",
            world_pt.point.x, world_pt.point.y, place_radius,
            place_max_reach_m_);
          continue;
        }
        place_candidates.push_back({u, v, world_pt});
      }
    }
  }

  // ── Hand-gate streak tracker ───────────────────────────────────────────────
  // When wait_for_hand_before_pick_ is on, the WAITING_FOR_HAND state needs
  // to know how long a place_class detection has been continuously present.
  // We only run the tracker in that state to avoid spurious streak updates
  // during the active pick/place cycle (a hand seen while the arm is over
  // the table doesn't count toward the next cycle's gate).
  if (wait_for_hand_before_pick_ && state_ == PnPState::WAITING_FOR_HAND) {
    bool saw_hand_this_frame = false;
    for (const auto & det : msg->detections) {
      if (det.results.empty()) continue;
      const auto & hyp = det.results[0].hypothesis;
      if (hyp.score < min_detection_score_) continue;
      const bool match = (place_class_ == "*") ||
        (std::find(place_class_set_.begin(), place_class_set_.end(),
                   hyp.class_id) != place_class_set_.end());
      if (match) { saw_hand_this_frame = true; break; }
    }
    if (saw_hand_this_frame) {
      const rclcpp::Time now = this->now();
      // Restart the streak if the gap since the last hand was longer than
      // hand_gate_stale_s_. Frame-debounce: tiny YOLO flickers (a missed
      // frame or two) don't break the streak.
      const bool streak_active = hand_streak_start_time_.nanoseconds() != 0;
      const double gap = (last_hand_seen_time_.nanoseconds() == 0)
        ? std::numeric_limits<double>::infinity()
        : (now - last_hand_seen_time_).seconds();
      if (!streak_active || gap > hand_gate_stale_s_) {
        hand_streak_start_time_ = now;
      }
      last_hand_seen_time_ = now;
    }
  }

  // Resolve place candidates: pick the one whose pixel is closest to the
  // principal point (camera_cx_, camera_cy_). When the wildcard is off and
  // only one detection passes the class filter this collapses to the
  // previous "first match wins" behaviour.
  if (!place_target_latched_ && place_mode_ == "detection"
      && !place_candidates.empty()) {
    const PlaceCandidate * best = nullptr;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (const auto & c : place_candidates) {
      double du = c.u - camera_cx_;
      double dv = c.v - camera_cy_;
      double d2 = du * du + dv * dv;
      if (d2 < best_d2) { best_d2 = d2; best = &c; }
    }

    if (place_dwell_time_s_ <= 0.0) {
      // Legacy immediate latch — used by existing kitkat/COCO place flows
      // where the place object is a static thing and there's no point
      // making the operator hold a duck still.
      place_target_ = best->world_pt;
      place_target_valid_ = true;
    } else {
      // Dwell-lock — mirror the wrist-lock state machine from
      // pose_inference_node.py. The operator must hold the most-central
      // hand within place_dwell_tolerance_m_ of itself for
      // place_dwell_time_s_ seconds before we commit. Movement beyond
      // tolerance restarts the dwell at the new position.
      const auto & bp = best->world_pt.point;
      if (!place_dwell_active_) {
        place_dwell_pos_   = best->world_pt;
        place_dwell_start_ = now();
        place_dwell_active_ = true;
        RCLCPP_INFO(get_logger(),
          "lock: candidate at (%.3f, %.3f) — dwell starting (%.1fs target)",
          bp.x, bp.y, place_dwell_time_s_);
      } else {
        const double dx = bp.x - place_dwell_pos_.point.x;
        const double dy = bp.y - place_dwell_pos_.point.y;
        const double drift = std::hypot(dx, dy);
        if (drift <= place_dwell_tolerance_m_) {
          const double elapsed = (now() - place_dwell_start_).seconds();
          if (elapsed >= place_dwell_time_s_) {
            place_target_ = best->world_pt;
            place_target_valid_ = true;
            if (!place_dwell_locked_) {
              RCLCPP_INFO(get_logger(),
                "lock: LOCKED at (%.3f, %.3f) — held %.1fs",
                bp.x, bp.y, elapsed);
              place_dwell_locked_ = true;
            }
          } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
              "lock: dwell %.2f/%.1fs at (%.3f, %.3f)",
              elapsed, place_dwell_time_s_, bp.x, bp.y);
          }
        } else {
          place_dwell_pos_   = best->world_pt;
          place_dwell_start_ = now();
          place_dwell_locked_ = false;
          RCLCPP_INFO(get_logger(),
            "lock: hand moved %.0f cm — dwell reset at (%.3f, %.3f)",
            drift * 100.0, bp.x, bp.y);
        }
      }
    }
  }

  if (pick_candidates.empty()) return;

  // In fixed-pick mode the operator-chosen XY is authoritative and the
  // background YOLO must NOT overwrite pick_target_ — otherwise the
  // state machine's DETECTING_PICK synthesis is silently bypassed
  // because pick_target_valid_ is already true by the time tick() runs.
  // Skip the entire pick-target update path here; the synthesis in
  // DETECTING_PICK will run unconditionally and use the fixed XY.
  if (use_fixed_pick_pose_) return;

  if (!pick_target_initialized_) {
    // Bootstrap: commit to exactly one candidate from this frame using the
    // configured selection rule.
    pick_target_ = select_pick_candidate(pick_candidates);
    pick_target_initialized_ = true;
    pick_detection_count_ = 1;
  } else {
    // Already tracking a specific object — only update the EMA with the
    // candidate closest to the current target, and only if it's within the
    // acceptance radius. Others in this frame are different objects.
    const geometry_msgs::msg::PointStamped * closest = nullptr;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (const auto & p : pick_candidates) {
      double dx = p.point.x - pick_target_.point.x;
      double dy = p.point.y - pick_target_.point.y;
      double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; closest = &p; }
    }
    if (closest && std::sqrt(best_d2) <= pick_accept_radius_m_) {
      pick_target_.point.x = pick_ema_alpha_ * closest->point.x +
                              (1.0 - pick_ema_alpha_) * pick_target_.point.x;
      pick_target_.point.y = pick_ema_alpha_ * closest->point.y +
                              (1.0 - pick_ema_alpha_) * pick_target_.point.y;
      pick_target_.header = closest->header;
      // Count once per frame (previously once per detection, which made
      // min_pick_detections_ trivially satisfiable with multi-object scenes).
      pick_detection_count_++;
    }
  }

  if (pick_detection_count_ >= min_pick_detections_) {
    pick_target_valid_ = true;
  }
}

geometry_msgs::msg::PointStamped PickAndPlaceNode::select_pick_candidate(
  const std::vector<geometry_msgs::msg::PointStamped> & candidates) const
{
  // Lower score wins.
  auto score = [this](const geometry_msgs::msg::PointStamped & p) -> double {
    if (pick_selection_mode_ == "rightmost") {
      // On this robot +Y = right; maximise y, so negate for "lower is better".
      return -p.point.y;
    }
    if (pick_selection_mode_ == "leftmost") {
      return p.point.y;  // -Y = left, so minimise y
    }
    if (pick_selection_mode_ == "nearest_to_x_axis") {
      return std::abs(p.point.y);
    }
    // default: nearest_to_base — Euclidean distance from origin
    return std::hypot(p.point.x, p.point.y);
  };

  const geometry_msgs::msg::PointStamped * best = &candidates[0];
  double best_score = score(candidates[0]);
  for (size_t i = 1; i < candidates.size(); ++i) {
    double s = score(candidates[i]);
    if (s < best_score) { best_score = s; best = &candidates[i]; }
  }
  RCLCPP_INFO(get_logger(),
    "Committed to pick candidate at (%.3f, %.3f) using mode '%s' "
    "(chose 1 of %zu)",
    best->point.x, best->point.y, pick_selection_mode_.c_str(),
    candidates.size());
  return *best;
}

// ── 3D projection ────────────────────────────────────────────────────────────

bool PickAndPlaceNode::project_to_world(
  double u, double v, double depth_m,
  geometry_msgs::msg::PointStamped & world_point)
{
  double X_cam = (u - camera_cx_) * depth_m / camera_fx_;
  double Y_cam = (v - camera_cy_) * depth_m / camera_fy_;
  double Z_cam = depth_m;

  geometry_msgs::msg::PointStamped point_cam;
  point_cam.header.stamp = this->now();
  point_cam.header.frame_id = camera_frame_;
  point_cam.point.x = X_cam;
  point_cam.point.y = Y_cam;
  point_cam.point.z = Z_cam;

  try {
    world_point = tf_buffer_->transform(point_cam, robot_base_frame_);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "TF transform failed: %s", ex.what());
    return false;
  }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

geometry_msgs::msg::Pose PickAndPlaceNode::make_down_pose(
  double x, double y, double z)
{
  // Clamp to workspace
  double dist = std::hypot(x, y);
  if (dist > max_reach_m_ && dist > 0.0) {
    double scale = max_reach_m_ / dist;
    x *= scale;
    y *= scale;
    RCLCPP_INFO(get_logger(), "  Clamped to workspace: (%.3f, %.3f)", x, y);
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  // Tool pointing straight down
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.707;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.707;
  return pose;
}

geometry_msgs::msg::Pose PickAndPlaceNode::make_place_pose(
  double x, double y, double z)
{
  // Re-use make_down_pose for positioning + workspace clamping, then (in
  // tf-mode) override the orientation to match the wrist-tracking demo's
  // down pose — fingers open along world +Y. Positional clamping already
  // ran inside make_down_pose, so this just swaps the quaternion.
  auto pose = make_down_pose(x, y, z);
  if (place_mode_ == "tf") {
    pose.orientation.x = 0.70710678;
    pose.orientation.y = 0.70710678;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
  }
  return pose;
}

bool PickAndPlaceNode::try_lookup_place_tf()
{
  if (place_tf_frame_.empty()) {
    return false;
  }
  try {
    auto tf = tf_buffer_->lookupTransform(
      robot_base_frame_, place_tf_frame_, tf2::TimePointZero);
    rclcpp::Time stamp(tf.header.stamp, RCL_ROS_TIME);
    double age_s = (now() - stamp).seconds();
    if (age_s > place_tf_max_age_s_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "  Place TF '%s' is %.2fs stale (> %.2fs) — still waiting",
        place_tf_frame_.c_str(), age_s, place_tf_max_age_s_);
      return false;
    }
    place_target_.header.stamp = now();
    place_target_.header.frame_id = robot_base_frame_;
    place_target_.point.x = tf.transform.translation.x;
    place_target_.point.y = tf.transform.translation.y;
    place_target_.point.z = tf.transform.translation.z;
    place_target_valid_ = true;
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "  Place TF '%s' not yet available (%s)",
      place_tf_frame_.c_str(), e.what());
    return false;
  }
}

bool PickAndPlaceNode::move_to_pose(const geometry_msgs::msg::Pose & pose)
{
  RCLCPP_INFO(get_logger(), "  Moving to (%.3f, %.3f, %.3f)",
    pose.position.x, pose.position.y, pose.position.z);

  move_group_->setPoseTarget(pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group_->plan(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(get_logger(), "  Planning failed (code: %d)", result.val);
    return false;
  }

  RCLCPP_INFO(get_logger(), "  Plan succeeded — executing");
  auto exec_result = move_group_->execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(get_logger(), "  Execution failed (code: %d)", exec_result.val);
    return false;
  }

  RCLCPP_INFO(get_logger(), "  Motion complete");
  return true;
}

bool PickAndPlaceNode::move_cartesian(const geometry_msgs::msg::Pose & pose)
{
  RCLCPP_INFO(get_logger(), "  Moving (Cartesian) to (%.3f, %.3f, %.3f)",
    pose.position.x, pose.position.y, pose.position.z);

  std::vector<geometry_msgs::msg::Pose> waypoints{pose};
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_->computeCartesianPath(waypoints, 0.005, trajectory);

  if (fraction < 0.9) {
    RCLCPP_WARN(get_logger(), "  Cartesian plan incomplete (%.0f%%) — falling back to joint-space",
      fraction * 100.0);
    return move_to_pose(pose);
  }

  RCLCPP_INFO(get_logger(), "  Cartesian plan %.0f%% — executing", fraction * 100.0);
  auto exec_result = move_group_->execute(trajectory);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(get_logger(), "  Cartesian execution failed (code: %d)", exec_result.val);
    return false;
  }

  RCLCPP_INFO(get_logger(), "  Motion complete");
  return true;
}

bool PickAndPlaceNode::gripper_open()
{
  RCLCPP_INFO(get_logger(), "  Opening gripper...");
  send_gripper_async(1);  // OPEN_GRIPPER
  return true;
}

bool PickAndPlaceNode::gripper_close()
{
  RCLCPP_INFO(get_logger(), "  Closing gripper...");
  send_gripper_async(2);  // CLOSE_GRIPPER
  return true;
}

bool PickAndPlaceNode::send_gripper_cmd(uint8_t /*cmd_type*/)
{
  // Unused — kept for interface compatibility
  return false;
}

void PickAndPlaceNode::send_gripper_async(uint8_t cmd_type)
{
  gripper_busy_ = true;
  gripper_succeeded_ = false;

  auto goal = ToolAction::Goal();
  goal.cmd.cmd_type = cmd_type;
  goal.cmd.tool_id = tool_id_;
  goal.cmd.max_torque_percentage = max_torque_pct_;
  goal.cmd.hold_torque_percentage = hold_torque_pct_;

  auto send_goal_options = rclcpp_action::Client<ToolAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&PickAndPlaceNode::on_gripper_goal_response, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&PickAndPlaceNode::on_gripper_result, this, std::placeholders::_1);

  gripper_client_->async_send_goal(goal, send_goal_options);
}

void PickAndPlaceNode::on_gripper_goal_response(GoalHandleTool::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "  Gripper goal rejected");
    gripper_busy_ = false;
    gripper_succeeded_ = false;
  }
}

void PickAndPlaceNode::on_gripper_result(const GoalHandleTool::WrappedResult & result)
{
  (void)result;
  RCLCPP_INFO(get_logger(), "  Gripper command complete");
  gripper_busy_ = false;
  gripper_succeeded_ = true;
}

// ── State machine ────────────────────────────────────────────────────────────

void PickAndPlaceNode::tick()
{
  if (state_ == PnPState::WAITING_FOR_INIT || state_ == PnPState::DONE) {
    return;
  }

  if (!move_group_ready_) return;

  auto prev_state = state_;

  switch (state_) {

    case PnPState::WAITING_FOR_HAND: {
      // Frame-debounced gate. Hand must be seen continuously
      // (no gap > hand_gate_stale_s_) for at least hand_gate_duration_s_
      // before the cycle is allowed to start. The streak tracker is
      // updated in detection_callback only while in this state.
      const rclcpp::Time now = this->now();
      if (last_hand_seen_time_.nanoseconds() == 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "Hand-gate: waiting for '%s' detection...", place_class_.c_str());
        return;
      }
      const double age = (now - last_hand_seen_time_).seconds();
      if (age > hand_gate_stale_s_) {
        if (hand_streak_start_time_.nanoseconds() != 0) {
          RCLCPP_INFO(get_logger(),
            "Hand-gate: streak broken (last seen %.2fs ago) — waiting for "
            "hand to reappear", age);
          hand_streak_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        return;
      }
      const double held = (hand_streak_start_time_.nanoseconds() == 0)
        ? 0.0 : (now - hand_streak_start_time_).seconds();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "Hand-gate: hand visible %.2f/%.1fs", held, hand_gate_duration_s_);
      if (held >= hand_gate_duration_s_) {
        RCLCPP_INFO(get_logger(),
          "=== Hand-gate: '%s' held for %.2fs — starting pick cycle ===",
          place_class_.c_str(), held);
        // Reset so the next entry into WAITING_FOR_HAND (post-drop) starts
        // a fresh streak — operator must re-show the hand.
        last_hand_seen_time_     = rclcpp::Time(0, 0, RCL_ROS_TIME);
        hand_streak_start_time_  = rclcpp::Time(0, 0, RCL_ROS_TIME);
        state_ = PnPState::DETECTING_PICK;
      }
      break;
    }

    case PnPState::DETECTING_PICK: {
      // Plan B: skip detection-based targeting and synthesize a fixed
      // pick pose. Used when camera calibration is too poor for reliable
      // back-projection but the hand-place flow still works. The arm
      // grasps at world (fixed_pick_x_, fixed_pick_y_, 0) on every cycle.
      if (use_fixed_pick_pose_ && !pick_target_valid_) {
        pick_target_.header.frame_id = robot_base_frame_;
        pick_target_.header.stamp    = now();
        pick_target_.point.x = fixed_pick_x_;
        pick_target_.point.y = fixed_pick_y_;
        pick_target_.point.z = 0.0;
        pick_target_valid_       = true;
        pick_target_initialized_ = true;
        pick_detection_count_    = min_pick_detections_;  // pre-satisfy debounce
        RCLCPP_INFO(get_logger(),
          "Fixed pick pose: synthesizing pick target at world (%.3f, %.3f)",
          fixed_pick_x_, fixed_pick_y_);
      }
      if (!pick_target_valid_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "Waiting for '%s' detection...", pick_class_.c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "=== PICK target '%s' at (%.3f, %.3f) — latching ===",
        pick_class_.c_str(), pick_target_.point.x, pick_target_.point.y);
      pick_target_latched_ = true;
      approach_pick_attempts_ = 0;
      state_ = PnPState::APPROACHING_PICK;
      break;
    }

    case PnPState::APPROACHING_PICK: {
      auto pose = make_down_pose(
        pick_target_.point.x + pick_offset_x_,
        pick_target_.point.y + pick_offset_y_, pick_hover_height_m_);
      approach_pick_attempts_++;
      if (move_to_pose(pose)) {
        approach_pick_attempts_ = 0;
        gripper_next_state_ = PnPState::DESCENDING_TO_PICK;
        state_ = PnPState::OPENING_GRIPPER;
      } else if (approach_pick_attempts_ >= max_approach_pick_attempts_) {
        RCLCPP_WARN(get_logger(),
          "Approach pick failed %d/%d times at (%.3f, %.3f, %.3f) — "
          "dropping latch and re-detecting. Move the '%s' closer to the "
          "robot base (current approach radius %.3fm) or raise "
          "max_approach_pick_attempts.",
          approach_pick_attempts_, max_approach_pick_attempts_,
          pose.position.x, pose.position.y, pose.position.z,
          pick_class_.c_str(),
          std::hypot(pose.position.x, pose.position.y));
        approach_pick_attempts_     = 0;
        pick_target_latched_        = false;
        pick_target_valid_          = false;
        pick_target_initialized_    = false;
        pick_detection_count_       = 0;
        state_ = PnPState::DETECTING_PICK;
      } else {
        RCLCPP_WARN(get_logger(),
          "Approach pick failed (%d/%d) — retrying with latched position",
          approach_pick_attempts_, max_approach_pick_attempts_);
        state_ = PnPState::APPROACHING_PICK;
      }
      break;
    }

    case PnPState::OPENING_GRIPPER: {
      if (gripper_busy_) {
        // Waiting for async gripper to finish
        return;
      }
      if (gripper_succeeded_) {
        state_ = gripper_next_state_;
        gripper_succeeded_ = false;
      } else {
        gripper_next_state_ = PnPState::DESCENDING_TO_PICK;
        gripper_open();
      }
      break;
    }

    case PnPState::DESCENDING_TO_PICK: {
      double safe_z = std::max(grasp_height_m_, grasp_min_height_m_);
      RCLCPP_INFO(get_logger(), "  Descending to z=%.4f (grasp_height_m=%.4f)",
        safe_z, grasp_height_m_);
      auto pose = make_down_pose(
        pick_target_.point.x + pick_offset_x_,
        pick_target_.point.y + pick_offset_y_, safe_z);
      if (move_cartesian(pose)) {
        state_ = PnPState::GRIPPING;
      } else {
        RCLCPP_WARN(get_logger(), "Descend to pick failed — lifting and retrying");
        auto lift = make_down_pose(
          pick_target_.point.x, pick_target_.point.y, pick_hover_height_m_);
        move_to_pose(lift);
        state_ = PnPState::APPROACHING_PICK;
      }
      break;
    }

    case PnPState::GRIPPING: {
      if (gripper_busy_) {
        return;
      }
      if (gripper_succeeded_) {
        grip_attempts_++;
        gripper_succeeded_ = false;
        if (grip_attempts_ >= max_grip_attempts_) {
          RCLCPP_INFO(get_logger(), "  Grip confirmed after %d attempts", grip_attempts_);
          grip_attempts_ = 0;
          state_ = PnPState::LIFTING;
        } else {
          RCLCPP_INFO(get_logger(), "  Grip attempt %d/%d — squeezing again",
            grip_attempts_, max_grip_attempts_);
          gripper_close();
        }
      } else {
        grip_attempts_ = 0;
        gripper_close();
      }
      break;
    }

    case PnPState::LIFTING: {
      auto pose = make_down_pose(
        pick_target_.point.x + pick_offset_x_,
        pick_target_.point.y + pick_offset_y_, lift_height_m_);
      if (move_to_pose(pose)) {
        RCLCPP_INFO(get_logger(), "=== Lift complete — verifying grip ===");
        state_ = PnPState::VERIFYING_GRIP;
      } else {
        RCLCPP_WARN(get_logger(), "Lift failed — retrying");
      }
      break;
    }

    case PnPState::VERIFYING_GRIP: {
      if (!verify_moved_clear_) {
        // Phase 1: Move arm away from pick area so camera can see the table
        auto clear_pose = make_down_pose(0.12, 0.0, lift_height_m_);
        if (move_to_pose(clear_pose)) {
          verify_moved_clear_ = true;
          pre_verify_pick_target_ = pick_target_;  // save original before unlatching
          // Reset pick detection so we can check fresh
          pick_target_latched_ = false;
          pick_target_initialized_ = false;
          pick_target_valid_ = false;
          pick_detection_count_ = 0;
          verify_start_time_ = now();
          // ── Reset hand-finding state ─────────────────────────────────
          // Customer-success observation (2 kitkats on mat + 1 in hand):
          // any hand detections accumulated during the pick cycle are
          // stale by the time we get here. The operator may have moved
          // their hand while the arm was working, so the dwell-centre
          // and lock state from before-the-pick lead to dropping at the
          // wrong place. Wiping these here means the upcoming
          // DETECTING_PLACE / ADAPTIVE_PLACE_HOVER starts from a clean
          // slate and only sees detections from the home-position
          // moment forward.
          place_dwell_active_      = false;
          place_dwell_locked_      = false;
          place_target_valid_      = false;
          place_target_latched_    = false;
          place_approach_failures_ = 0;
          adaptive_has_commanded_  = false;
          RCLCPP_INFO(get_logger(),
            "  Arm at home with kitkat — hand-find state reset");
          RCLCPP_INFO(get_logger(),
            "  Arm clear — watching for '%s' near (%.3f, %.3f) "
            "within r=%.3fm for %.1fs...",
            pick_class_.c_str(),
            pre_verify_pick_target_.point.x, pre_verify_pick_target_.point.y,
            verify_radius_m_, verify_duration_s_);
        } else {
          RCLCPP_WARN(get_logger(), "  Failed to move clear — skipping verification");
          verify_moved_clear_ = false;
          if (place_mode_ == "fixed") {
            place_target_.header.frame_id = robot_base_frame_;
            place_target_.point.x = fixed_drop_x_ + place_offset_x_m_;
            place_target_.point.y = fixed_drop_y_ + place_offset_y_m_;
            place_target_.point.z = 0.0;
            place_target_valid_ = true;
            place_target_latched_ = true;
            state_ = PnPState::APPROACHING_PLACE;
          } else {
            // detection or tf — wait in DETECTING_PLACE
            state_ = PnPState::DETECTING_PLACE;
          }
        }
        break;
      }

      // Phase 2: Wait and check if object is still visible on the table
      double elapsed = (now() - verify_start_time_).seconds();

      // Verify outcome handling. Previously a "failed" verify retried the
      // pick, which went via OPENING_GRIPPER — with the consequence that if
      // the gripper was actually holding an object and verify fired falsely
      // (e.g. a neighbour object inside the verify radius), the object
      // would be *dropped back on the table at the pick spot*. That was the
      // observed "picks up then drops back" failure mode.
      //
      // Instead we now always proceed to place on any verify outcome.
      // - Grip really succeeded → object placed at drop correctly.
      // - Grip really failed    → empty release at drop; the scene-scan loop
      //                            will pick the object on the next cycle.
      // - Verify was wrong      → no regression vs the real outcome.

      auto go_to_place = [this](const char * reason) {
        RCLCPP_INFO(get_logger(),
          "  Verify: %s — proceeding to place.", reason);
        verify_moved_clear_ = false;
        if (place_mode_ == "fixed") {
          place_target_.header.stamp = now();
          place_target_.header.frame_id = robot_base_frame_;
          place_target_.point.x = fixed_drop_x_ + place_offset_x_m_;
          place_target_.point.y = fixed_drop_y_ + place_offset_y_m_;
          place_target_.point.z = 0.0;
          place_target_valid_ = true;
          place_target_latched_ = true;
          state_ = PnPState::APPROACHING_PLACE;
        } else {
          // "detection" or "tf" — wait for a target in DETECTING_PLACE.
          state_ = PnPState::DETECTING_PLACE;
        }
      };

      // Pick spot looks occupied → grip failed. Two responses:
      //   retry_on_failed_verify_=false (default): trust the grip and go to
      //     place anyway. Safe when the gripper might be holding a real
      //     object that just happens to be near another detected object in
      //     a multi-object scene (drop-back avoidance).
      //   retry_on_failed_verify_=true (kitkat profile): the bar slipped
      //     out and is sitting on the table — re-scan and try again. The
      //     gripper is opened first AT THE CURRENT (verify) pose, which is
      //     away from the pick spot, so even if this was a false positive
      //     and we did hold something, we'd drop it at the lift height
      //     rather than back on the pick.
      if (pick_detection_count_ >= 3) {
        // Fixed-pick mode caps retries because the verify check itself
        // is unreliable when camera calibration is poor (the bar may
        // back-project to anywhere within ~20 cm of its true location).
        // Without a cap, a stuck verify could loop forever. After
        // max_fixed_pick_attempts_, fall through to the "place anyway"
        // branch — better to drop empty than to spin.
        bool retry_allowed = retry_on_failed_verify_ && loop_mode_;
        if (retry_allowed && use_fixed_pick_pose_) {
          fixed_pick_attempts_++;
          if (fixed_pick_attempts_ >= max_fixed_pick_attempts_) {
            RCLCPP_WARN(get_logger(),
              "  Fixed-pick: %d/%d attempts exhausted — proceeding to "
              "place with current gripper contents",
              fixed_pick_attempts_, max_fixed_pick_attempts_);
            retry_allowed = false;
          }
        }
        if (retry_allowed) {
          if (use_fixed_pick_pose_) {
            RCLCPP_WARN(get_logger(),
              "  '%s' detected near (%.3f, %.3f) during verify — "
              "grip FAILED, retrying pick (attempt %d/%d)",
              pick_class_.c_str(),
              pre_verify_pick_target_.point.x, pre_verify_pick_target_.point.y,
              fixed_pick_attempts_, max_fixed_pick_attempts_);
          } else {
            RCLCPP_WARN(get_logger(),
              "  '%s' detected near (%.3f, %.3f) during verify — "
              "grip FAILED, retrying pick",
              pick_class_.c_str(),
              pre_verify_pick_target_.point.x, pre_verify_pick_target_.point.y);
          }
          pick_detection_count_ = 0;
          pick_target_initialized_ = false;
          // Reset the verify-clear latch so the NEXT cycle's verify state
          // re-runs Phase 1 (move to clear pose + restart timer) instead
          // of falling through to Phase 2 with a stale verify_start_time_
          // and immediately declaring "grip VERIFIED!" without actually
          // watching.
          verify_moved_clear_ = false;
          // Open the gripper before re-scanning so a subsequent pick has a
          // clean starting state. Routes through OPENING_GRIPPER state so
          // we wait for the action to complete before scanning.
          gripper_next_state_ = PnPState::DETECTING_PICK;
          state_ = PnPState::OPENING_GRIPPER;
          gripper_open();
        } else {
          RCLCPP_WARN(get_logger(),
            "  '%s' detected near (%.3f, %.3f) during verify — "
            "but placing anyway to avoid drop-back",
            pick_class_.c_str(),
            pre_verify_pick_target_.point.x, pre_verify_pick_target_.point.y);
          go_to_place("pick spot still looks occupied");
        }
        break;
      }

      // Still waiting
      if (elapsed < verify_duration_s_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "  Verifying grip... %.1fs / %.1fs (detections: %d)",
          elapsed, verify_duration_s_, pick_detection_count_);
        return;
      }

      // Timeout reached.
      if (pick_target_initialized_) {
        RCLCPP_WARN(get_logger(),
          "  '%s' seen %d time(s) near pick spot — uncertain but placing anyway",
          pick_class_.c_str(), pick_detection_count_);
        go_to_place("uncertain but trusting grip");
      } else {
        RCLCPP_INFO(get_logger(),
          "  No '%s' near pick spot — grip VERIFIED!",
          pick_class_.c_str());
        go_to_place("grip verified");
      }
      break;
    }

    case PnPState::DETECTING_PLACE: {
      // Adaptive entry: as soon as we have any hand candidate (dwell active)
      // and the user opted into adaptive mode, skip the latched-target flow
      // and go straight to ADAPTIVE_PLACE_HOVER, where the arm follows the
      // hand around until the dwell finally satisfies. Only applies in
      // detection mode — TF-mode place keeps its existing instant-latch
      // behaviour because the wrist transform is already smooth.
      if (adaptive_place_enabled_ && place_mode_ == "detection"
          && place_dwell_active_) {
        RCLCPP_INFO(get_logger(),
          "Adaptive place: hand detected — entering hover-track");
        adaptive_has_commanded_ = false;
        state_ = PnPState::ADAPTIVE_PLACE_HOVER;
        return;
      }
      // In TF mode the place target isn't populated by the detection
      // callback — poll the TF tree until the configured frame is fresh.
      if (place_mode_ == "tf" && !place_target_valid_) {
        try_lookup_place_tf();
      }
      if (!place_target_valid_) {
        if (place_mode_ == "tf") {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "Waiting for place TF '%s' to become available...",
            place_tf_frame_.c_str());
        } else {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "Waiting for '%s' detection...", place_class_.c_str());
        }
        return;
      }
      place_target_.point.x += place_offset_x_m_;
      place_target_.point.y += place_offset_y_m_;
      if (place_mode_ == "tf") {
        RCLCPP_INFO(get_logger(),
          "=== PLACE target via TF '%s' at (%.3f, %.3f, %.3f) — latching ===",
          place_tf_frame_.c_str(),
          place_target_.point.x, place_target_.point.y, place_target_.point.z);
      } else {
        RCLCPP_INFO(get_logger(),
          "=== PLACE target '%s' at (%.3f, %.3f) — latching ===",
          place_class_.c_str(), place_target_.point.x, place_target_.point.y);
      }
      place_target_latched_ = true;
      place_approach_failures_ = 0;
      state_ = PnPState::APPROACHING_PLACE;
      break;
    }

    case PnPState::ADAPTIVE_PLACE_HOVER: {
      // No hand seen yet (or the dwell was reset to inactive somewhere) —
      // park until the detection callback brings one back. Updates to
      // place_dwell_pos_ / place_dwell_locked_ continue to flow through
      // the existing dwell logic in detection_callback.
      if (!place_dwell_active_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "Adaptive: waiting for '%s' detection to (re)appear...",
          place_class_.c_str());
        break;
      }

      // Apply the same XY bias that DETECTING_PLACE / APPROACHING_PLACE use
      // so the arm hovers above the same effective point we'd lock onto in
      // non-adaptive mode (no surprise offsets when transitioning to descend).
      const double dwell_x_biased = place_dwell_pos_.point.x + place_offset_x_m_;
      const double dwell_y_biased = place_dwell_pos_.point.y + place_offset_y_m_;

      // Reach gate. The dwell-centre can drift outside the place reach
      // envelope; we just wait for it to come back rather than throwing the
      // user a planning failure. Matches DETECTING_PLACE's behaviour.
      const double dwell_radius = std::hypot(dwell_x_biased, dwell_y_biased);
      if (dwell_radius > place_max_reach_m_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "Adaptive: dwell-centre at (%.3f, %.3f) radius %.2fm > place_max_reach "
          "%.2fm — waiting for hand to come closer",
          dwell_x_biased, dwell_y_biased, dwell_radius, place_max_reach_m_);
        break;
      }

      // How far is the arm currently from the dwell-centre?
      // We deliberately use the LAST COMMANDED hover position (not
      // move_group_->getCurrentPose()) as the arm-pose proxy. The Pi-side
      // bridge to ned-ros2-driver doesn't reliably satisfy the move_group
      // state monitor's "fresh joint state within 1 s" check —
      // getCurrentPose() then returns a zero-initialised pose and we'd
      // wrongly conclude the arm is 41 cm away (= the distance from world
      // origin to the dwell-centre), preventing the commit branch from
      // ever firing. The commanded-position proxy is semantically what
      // we want anyway: "did our last move command finish at the dwell
      // position?" If yes, arm is there; if no, we still owe it a chase.
      const double arm_to_dwell = adaptive_has_commanded_
        ? std::hypot(
            adaptive_last_commanded_target_.point.x - dwell_x_biased,
            adaptive_last_commanded_target_.point.y - dwell_y_biased)
        : std::numeric_limits<double>::infinity();

      // Commit: hand has been stable for place_dwell_time_s_ (dwell_locked_)
      // AND the arm has caught up. Lock the target and rejoin the normal
      // descent flow via APPROACHING_PLACE (which does a final tiny align).
      if (place_dwell_locked_ && arm_to_dwell <= adaptive_arm_close_m_) {
        RCLCPP_INFO(get_logger(),
          "Adaptive: hand stable + arm hovering (%.0f cm away) — committing "
          "descent on (%.3f, %.3f)",
          arm_to_dwell * 100.0, dwell_x_biased, dwell_y_biased);
        // Mirror the latch path of DETECTING_PLACE.
        place_target_ = place_dwell_pos_;
        place_target_.point.x = dwell_x_biased;
        place_target_.point.y = dwell_y_biased;
        place_target_valid_   = true;
        place_target_latched_ = true;
        place_approach_failures_ = 0;
        state_ = PnPState::APPROACHING_PLACE;
        break;
      }

      // Decide whether to chase: re-plan + move only if the dwell-centre has
      // drifted from the LAST commanded hover by more than follow_threshold
      // (small dead-zone so a steady but jittery hand doesn't keep moving
      // the arm). The first iteration is forced (adaptive_has_commanded_
      // starts false).
      const double drift_from_commanded = adaptive_has_commanded_
        ? std::hypot(
            dwell_x_biased - adaptive_last_commanded_target_.point.x,
            dwell_y_biased - adaptive_last_commanded_target_.point.y)
        : std::numeric_limits<double>::infinity();
      if (drift_from_commanded <= adaptive_follow_threshold_m_) {
        // Hand close to where we last commanded — just wait for the dwell to
        // accumulate (no new motion needed). Throttled status log so the
        // operator can see the gate is counting up.
        const double held = (now() - place_dwell_start_).seconds();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "Adaptive: tracking — dwell %.1f/%.1fs, arm %.0fcm from hand",
          held, place_dwell_time_s_, arm_to_dwell * 100.0);
        break;
      }

      // Chase: plan + execute hover move. Synchronous (~3 s on Pi); during
      // this the dwell tracker keeps running in detection_callback and may
      // reset if the user keeps moving — that's fine, we'll re-plan on the
      // next tick.
      double stack_dz = drops_completed_ * drop_stack_increment_m_;
      double hover_z  = hover_height_m_ + stack_dz;
      auto hover_pose = make_place_pose(dwell_x_biased, dwell_y_biased, hover_z);
      RCLCPP_INFO(get_logger(),
        "Adaptive: tracking — moving hover to (%.3f, %.3f, %.3f) "
        "(dwell drifted %.0f cm)",
        dwell_x_biased, dwell_y_biased, hover_z,
        drift_from_commanded * 100.0);
      if (move_to_pose(hover_pose)) {
        adaptive_last_commanded_target_ = place_dwell_pos_;
        adaptive_last_commanded_target_.point.x = dwell_x_biased;
        adaptive_last_commanded_target_.point.y = dwell_y_biased;
        adaptive_has_commanded_ = true;
      } else {
        RCLCPP_WARN(get_logger(),
          "Adaptive: hover plan/exec failed — will retry next tick");
      }
      break;
    }

    case PnPState::APPROACHING_PLACE: {
      double stack_dz = drops_completed_ * drop_stack_increment_m_;
      double approach_z = (place_mode_ == "tf")
        ? place_target_.point.z + place_tf_hover_offset_m_
        : hover_height_m_ + stack_dz;
      auto pose = make_place_pose(
        place_target_.point.x, place_target_.point.y, approach_z);
      if (move_to_pose(pose)) {
        place_approach_failures_ = 0;
        // In TF mode we skip DESCENDING_TO_PLACE — the approach pose is
        // already above the operator's hand, and that extra short descent
        // (5 cm-ish lower, same XY) was the main IK-pinch failure mode.
        // Opening the gripper from the approach height drops the duck
        // straight into the hand just as reliably.
        if (place_mode_ == "tf") {
          RCLCPP_INFO(get_logger(),
            "  TF-place: at approach pose above hand — opening gripper here");
          state_ = PnPState::RELEASING;
        } else {
          state_ = PnPState::DESCENDING_TO_PLACE;
        }
      } else {
        place_approach_failures_++;
        constexpr int kPlaceApproachMaxFailures = 3;
        if (place_approach_failures_ >= kPlaceApproachMaxFailures) {
          RCLCPP_WARN(get_logger(),
            "place: latched target (%.3f, %.3f) is unreachable after %d "
            "plan attempts — discarding lock. Bring hand closer (current "
            "hover Z=%.2f m gives ~%.2f m reach radius).",
            place_target_.point.x, place_target_.point.y,
            place_approach_failures_,
            hover_height_m_,
            std::sqrt(std::max(0.0, 0.44 * 0.44
                               - hover_height_m_ * hover_height_m_)));
          // Drop the lock and re-enter DETECTING_PLACE. The dwell state
          // machine will require the operator to hold a fresh steady
          // position before another lock forms — ideally somewhere
          // reachable this time. The bar stays in the gripper.
          place_target_latched_   = false;
          place_target_valid_     = false;
          place_dwell_active_     = false;
          place_dwell_locked_     = false;
          place_approach_failures_ = 0;
          state_ = PnPState::DETECTING_PLACE;
        } else {
          RCLCPP_WARN(get_logger(),
            "Approach place failed (%d/%d) — retrying with latched position",
            place_approach_failures_, kPlaceApproachMaxFailures);
          state_ = PnPState::APPROACHING_PLACE;
        }
      }
      break;
    }

    case PnPState::DESCENDING_TO_PLACE: {
      double stack_dz = drops_completed_ * drop_stack_increment_m_;
      double drop_z = (place_mode_ == "tf")
        ? place_target_.point.z + place_tf_drop_offset_m_
        : place_drop_height_m_ + stack_dz;
      auto pose = make_place_pose(
        place_target_.point.x, place_target_.point.y, drop_z);
      if (move_to_pose(pose)) {
        RCLCPP_INFO(get_logger(),
          "  Drop #%d — placing at z=%.4f",
          drops_completed_ + 1, drop_z);
        state_ = PnPState::RELEASING;
      } else {
        RCLCPP_WARN(get_logger(), "Descend to place failed — retrying");
        double lift_z = (place_mode_ == "tf")
          ? place_target_.point.z + place_tf_hover_offset_m_
          : hover_height_m_ + stack_dz;
        auto lift = make_place_pose(
          place_target_.point.x, place_target_.point.y, lift_z);
        move_to_pose(lift);
        state_ = PnPState::APPROACHING_PLACE;
      }
      break;
    }

    case PnPState::RELEASING: {
      if (gripper_busy_) {
        return;
      }
      if (gripper_succeeded_) {
        // An object has now been placed; bump the stack counter so the
        // retract (and all subsequent approaches/descents/retracts) clear it.
        drops_completed_++;
        state_ = PnPState::RETRACTING;
        gripper_succeeded_ = false;
      } else {
        gripper_open();
      }
      break;
    }

    case PnPState::RETRACTING: {
      double stack_dz = drops_completed_ * drop_stack_increment_m_;
      // TF mode: after dropping into the hand, return to the same clear
      // pose we use during grip verification (0.12, 0.0, lift_height_m_)
      // and with the canonical down orientation (via make_down_pose). This
      // is known-good kinematically and positions the arm to rescan the
      // table for the next duck.
      geometry_msgs::msg::Pose pose;
      // Hand-place mode (any form — wildcard or specific gesture(s)) shares
      // TF mode's retract pose: NED2's reach envelope shrinks fast as Z
      // rises, and "straight up over the operator's palm" at lift_height_m_
      // frequently lands outside it, causing RETRACTING to spin on
      // Planning failed (code: 99999). Hand mode is identified by the
      // launch setting a shallower place_assumed_depth_m_ (e.g. 0.79 m
      // for a hand at Z=0.20 vs 0.99 m for the table) — a reliable
      // implicit signal that the place hover is above the table plane.
      const bool place_above_table =
        place_assumed_depth_m_ < assumed_depth_m_ - 0.01;
      if (place_mode_ == "tf" || place_above_table) {
        pose = make_down_pose(0.12, 0.0, lift_height_m_);
      } else {
        double retract_z = lift_height_m_ + stack_dz;
        pose = make_place_pose(
          place_target_.point.x, place_target_.point.y, retract_z);
      }
      if (move_to_pose(pose)) {
        RCLCPP_INFO(get_logger(),
          "\n======================================\n"
          "  PICK AND PLACE COMPLETE!\n"
          "  Picked '%s', placed at (%.3f, %.3f)\n"
          "======================================",
          pick_class_.c_str(),
          place_target_.point.x, place_target_.point.y);
        if (loop_mode_) {
          // Reset per-cycle state and scan the table for more targets.
          pick_target_valid_ = false;
          pick_target_initialized_ = false;
          pick_target_latched_ = false;
          pick_detection_count_ = 0;
          // In fixed-drop mode we synthesised place_target_; don't keep it
          // latched or we'll skip looking for the real next pick.
          if (!use_fixed_drop_pose_) {
            place_target_valid_ = false;
            place_target_latched_ = false;
            // Restart dwell from scratch for the next drop. Otherwise the
            // operator's hand from the previous cycle would count toward
            // the next lock immediately, which feels wrong if they've
            // moved between cycles.
            place_dwell_active_ = false;
            place_dwell_locked_ = false;
            // Adaptive tracker also resets per cycle — first move of the
            // next cycle is unconditional, regardless of where the user's
            // hand was during the previous drop.
            adaptive_has_commanded_ = false;
          }
          loop_moved_clear_ = false;
          // Fixed-pick attempt counter resets per cycle — the next loop
          // iteration gets a fresh budget of max_fixed_pick_attempts_
          // tries before giving up.
          fixed_pick_attempts_ = 0;
          // Hand-gated mode skips the table-empty scan — the operator
          // is the trigger, not the scene state. Each pick cycle waits
          // for a fresh hand signal before starting again.
          //
          // Fixed-pick mode ALSO skips the scene scan: there's no point
          // looking for a new bar in /detections because the pick branch
          // of the detection callback is short-circuited in this mode
          // (pick_target_valid_ would stay false forever and the loop
          // would idle). We go straight back to DETECTING_PICK so the
          // fixed-pose synthesis there sets up the next cycle.
          state_ =
            wait_for_hand_before_pick_ ? PnPState::WAITING_FOR_HAND :
            use_fixed_pick_pose_       ? PnPState::DETECTING_PICK   :
                                          PnPState::LOOP_CHECKING_SCENE;
        } else {
          state_ = PnPState::DONE;
        }
      } else {
        RCLCPP_WARN(get_logger(), "Retract failed — retrying");
      }
      break;
    }

    case PnPState::LOOP_CHECKING_SCENE: {
      if (!loop_moved_clear_) {
        // RETRACTING has already reset the pick-target state AND lifted the
        // arm to the drop pile (typically far outside the camera FOV, e.g.
        // y = -0.34 for thin flat objects). An extra "clear" move over the workspace
        // centre would actually OCCLUDE the remaining objects — so we just
        // start the scan immediately from wherever retract left us.
        loop_moved_clear_ = true;
        loop_scan_start_time_ = now();
        RCLCPP_INFO(get_logger(),
          "Loop: scanning %.1fs for more '%s' (arm parked at drop pile)...",
          loop_scene_timeout_s_, pick_class_.c_str());
        break;
      }

      double elapsed = (now() - loop_scan_start_time_).seconds();

      // Found another target → go straight back into the pick sequence.
      if (pick_target_valid_) {
        RCLCPP_INFO(get_logger(),
          "Loop: found another '%s' at (%.3f, %.3f) — starting next cycle",
          pick_class_.c_str(),
          pick_target_.point.x, pick_target_.point.y);
        loop_moved_clear_ = false;
        state_ = PnPState::DETECTING_PICK;
        break;
      }

      if (elapsed < loop_scene_timeout_s_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "Loop: scanning... %.1fs / %.1fs (detections: %d)",
          elapsed, loop_scene_timeout_s_, pick_detection_count_);
        return;
      }

      // Scan window expired with no valid target — table looks clear for now.
      // Don't terminate: reset the scan timer and keep watching. Objects
      // added to the scene later (or detector misses that clear up) will
      // resume the pipeline without needing a launch restart. Ctrl-C to
      // actually stop.
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Loop: no '%s' detected in %.1fs — idling and re-scanning "
        "(partial detections during window: %d)",
        pick_class_.c_str(), loop_scene_timeout_s_, pick_detection_count_);
      loop_scan_start_time_ = now();
      pick_target_valid_       = false;
      pick_target_initialized_ = false;
      pick_target_latched_     = false;
      pick_detection_count_    = 0;
      break;
    }

    default:
      break;
  }

  if (state_ != prev_state) {
    RCLCPP_INFO(get_logger(), "State: %s -> %s",
      state_name(prev_state), state_name(state_));
  }
}

}  // namespace metis_moveit2_perception

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<metis_moveit2_perception::PickAndPlaceNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread init_thread([&node]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    node->initialize();
  });
  init_thread.detach();

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
