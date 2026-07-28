// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Declares PickAndPlaceNode and its PnPState machine for vision guided pick
// and place on the NED2 arm.

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <niryo_ned_ros2_interfaces/action/tool.hpp>

namespace metis_moveit2_perception
{

enum class PnPState
{
  WAITING_FOR_INIT,
  // Optional pre-pick gate (enabled by wait_for_hand_before_pick_). Robot
  // idles in this state until a place_class detection (palm / stop / ...)
  // has been continuously visible for hand_gate_duration_s_. Only then
  // does it transition to DETECTING_PICK and start picking. In loop mode
  // it returns here after each successful drop so each cycle is operator-
  // triggered ("show palm to request a bar").
  WAITING_FOR_HAND,
  DETECTING_PICK,
  APPROACHING_PICK,
  OPENING_GRIPPER,
  DESCENDING_TO_PICK,
  GRIPPING,
  LIFTING,
  VERIFYING_GRIP,
  DETECTING_PLACE,
  // Optional adaptive place (enabled by adaptive_place_enabled_). Entered
  // from DETECTING_PLACE as soon as a hand is first seen, BEFORE the dwell
  // satisfies. The arm follows the hand around (re-planning to hover above
  // the current dwell-centre whenever it drifts > adaptive_follow_threshold_m_
  // from the last commanded position), and the descent only commits when
  // the existing dwell-lock satisfies AND the arm has caught up to within
  // adaptive_arm_close_m_. After commit the flow rejoins APPROACHING_PLACE.
  ADAPTIVE_PLACE_HOVER,
  APPROACHING_PLACE,
  DESCENDING_TO_PLACE,
  RELEASING,
  RETRACTING,
  LOOP_CHECKING_SCENE,  // loop mode: arm parked, watch for remaining objects
  DONE
};

class PickAndPlaceNode : public rclcpp::Node
{
public:
  explicit PickAndPlaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void initialize();

private:
  using ToolAction = niryo_ned_ros2_interfaces::action::Tool;
  using GoalHandleTool = rclcpp_action::ClientGoalHandle<ToolAction>;

  // Detection callback — caches latest positions for pick and place classes
  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  // State machine tick — called by timer
  void tick();

  // Helper: project pixel (u,v) to 3D world point. depth_m is the
  // camera-to-object distance assumed by the pinhole model — pick objects
  // (kitkat on the table) and place targets (a hand held above the table)
  // sit at different Z, so callers pass the depth appropriate to each.
  bool project_to_world(double u, double v, double depth_m,
                        geometry_msgs::msg::PointStamped & world_point);

  // Helper: build a pose at (x, y, z) with tool-pointing-down orientation
  geometry_msgs::msg::Pose make_down_pose(double x, double y, double z);

  // Helper: build the place-phase pose. Same as make_down_pose, except
  // when place_mode_ == "tf" the orientation is swapped for the "fingers
  // open along world +Y" quaternion used in the wrist-tracking demo.
  geometry_msgs::msg::Pose make_place_pose(double x, double y, double z);

  // Helper: look up the place TF (when place_mode_ == "tf"). Populates
  // place_target_ and sets place_target_valid_=true if a fresh transform
  // is available. Returns true on success.
  bool try_lookup_place_tf();

  // Helper: move arm to a pose, returns true on success
  bool move_to_pose(const geometry_msgs::msg::Pose & pose);

  // Helper: move arm along a straight Cartesian line (for descent/ascent)
  bool move_cartesian(const geometry_msgs::msg::Pose & pose);

  // Helper: send gripper command, returns true on success
  bool gripper_open();
  bool gripper_close();
  bool send_gripper_cmd(uint8_t cmd_type);

  // Helper: from a set of pick-class detections in one frame, choose the
  // single one we want to commit to using pick_selection_mode_.
  geometry_msgs::msg::PointStamped select_pick_candidate(
    const std::vector<geometry_msgs::msg::PointStamped> & candidates) const;

  // Parameters
  std::string pick_class_;
  std::string place_class_;
  // place_class_ parsed into individual classes (comma-separated input
  // like "palm,fist"). Empty if place_class_ == "*" (= match any).
  // Built once in initialize() to avoid re-parsing every detection.
  std::vector<std::string> place_class_set_;
  double camera_fx_, camera_fy_, camera_cx_, camera_cy_;
  double assumed_depth_m_;
  // Separate back-projection depth used only for the *place* candidate.
  // Pick objects lie on the table (depth ≈ camera height), but a hand
  // held above the table is closer to the camera, so the place class
  // needs a smaller depth for accurate XY. Falls back to assumed_depth_m_
  // if the parameter is left at its sentinel (≤ 0).
  double place_assumed_depth_m_;
  // Image dimensions + ignore-edge margin used when picking the "best"
  // place candidate (closest to image principal point, edges discarded).
  // image_width/height default to 1920x1080 (Pi C920); margin defaults to
  // 0 which disables the edge gate. Set non-zero from the launch when
  // running the hand-place profile so reflections/half-frame hands at
  // the image border don't latch as the drop target.
  int    image_width_;
  int    image_height_;
  int    place_edge_margin_px_;

  // Place dwell-lock. When place_dwell_time_s_ > 0 the place target is
  // not latched immediately. Instead the most-central candidate must
  // stay inside place_dwell_tolerance_m_ for the dwell duration before
  // becoming the place target. Mirrors the wrist lock state machine in
  // pose_inference_node.py. 0 disables (legacy first-frame latch).
  double place_dwell_time_s_;
  double place_dwell_tolerance_m_;
  // Independent reach gate for *place* candidates. The arm's reachable
  // horizontal radius shrinks rapidly at high Z (e.g. ~0.24 m at Z=0.37
  // vs ~0.44 m at the table) so a hand-place target that's fine for the
  // pick envelope can still be unreachable at the place hover. -1 sentinel
  // = "use max_reach_m_". Set tighter in hand mode (~0.30 m) so the
  // operator's hand has to be inside the high-Z envelope before the lock
  // forms.
  double place_max_reach_m_;
  bool   place_dwell_active_ = false;
  // Goes true the frame the dwell first satisfies, so we log "LOCKED" once
  // instead of every detection frame for the next 30 seconds. Reset when
  // the dwell breaks (operator moves hand out of tolerance) or when we
  // loop back from a successful drop.
  bool   place_dwell_locked_ = false;
  geometry_msgs::msg::PointStamped place_dwell_pos_;
  rclcpp::Time place_dwell_start_;

  // Consecutive APPROACHING_PLACE plan failures. After 3 in a row we
  // assume the latched target is genuinely IK-unreachable, drop the
  // lock, and return to DETECTING_PLACE so the operator's next steady
  // hand position (hopefully closer to the base) can form a fresh lock.
  // Resets to 0 on a successful approach or on a fresh latch from
  // DETECTING_PLACE.
  int    place_approach_failures_ = 0;

  // Pick-side hover height. Defaults to hover_height_m_ when left at the
  // sentinel (≤ 0), so existing launches don't change behaviour. Set
  // smaller than hover_height_m_ in the hand-place profile, where
  // hover_height_m_ is the *place* hover (above an operator's palm at
  // Z≈0.20 m) and would be wastefully high for the table pick.
  double pick_hover_height_m_;
  std::string camera_frame_;
  std::string robot_base_frame_;
  double max_reach_m_;
  // World-frame pick ROI. Pick-class detections whose back-projected (X, Y)
  // lies outside [pick_roi_x_min, pick_roi_x_max] × [pick_roi_y_min, pick_roi_y_max]
  // are silently rejected. Only the pick branch is gated — place / gesture
  // candidates can be anywhere in frame because that's where the operator
  // is standing. Disabled by default; enabled by the kitkat-YOLO launch path
  // (the HSV path already gates upstream via red_detector's image mask).
  bool   pick_roi_enabled_;
  double pick_roi_x_min_m_, pick_roi_x_max_m_;
  double pick_roi_y_min_m_, pick_roi_y_max_m_;

  // Pre-pick hand-gate. When wait_for_hand_before_pick_ is true the state
  // machine inserts a WAITING_FOR_HAND step before every DETECTING_PICK
  // entry (initial + each loop cycle). The gate fires when a place_class
  // detection has been continuously present for hand_gate_duration_s_
  // (counted as "last detection younger than hand_gate_stale_s_") —
  // single-frame YOLO flickers don't trigger it.
  bool   wait_for_hand_before_pick_;
  double hand_gate_duration_s_;
  double hand_gate_stale_s_;
  // Streak tracker — nanoseconds()==0 means "no streak active". Reset on
  // entering WAITING_FOR_HAND so a hand left over from the previous cycle
  // doesn't auto-trigger the next pick.
  rclcpp::Time last_hand_seen_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time hand_streak_start_time_{0, 0, RCL_ROS_TIME};

  // Adaptive place mode (hover-tracking before commit).
  bool   adaptive_place_enabled_;
  double adaptive_follow_threshold_m_;
  double adaptive_arm_close_m_;
  // Last hover XY we commanded the arm to during adaptive tracking.
  // Used to decide whether the dwell-centre has drifted enough to warrant
  // a fresh plan-and-move. Reset every loop iteration so each cycle starts
  // with a forced first move.
  geometry_msgs::msg::PointStamped adaptive_last_commanded_target_;
  bool   adaptive_has_commanded_{false};
  double hover_height_m_;
  double grasp_height_m_;
  double place_drop_height_m_;
  double lift_height_m_;
  double pick_offset_x_;
  double pick_offset_y_;
  double min_detection_score_;
  double pick_ema_alpha_;
  int    tool_id_;
  uint8_t max_torque_pct_;
  uint8_t hold_torque_pct_;

  // Extensions for thin / multi-instance objects
  double grasp_min_height_m_;    // safety floor for Z during descent
  double verify_duration_s_;     // how long to watch for re-appearance
  double verify_radius_m_;       // detections within this radius of original
  bool   retry_on_failed_verify_;// failed verify → re-scan instead of place
                                 // pick spot count as "still there"
  int    max_grip_attempts_;     // number of squeeze retries before trusting
  int    max_approach_pick_attempts_;  // plan-retry cap for APPROACHING_PICK
                                       // before dropping the latch
  bool   use_fixed_drop_pose_;   // skip DETECTING_PLACE and drop at a fixed XY
  double fixed_drop_x_;
  double fixed_drop_y_;

  // Plan B pick fallback. When use_fixed_pick_pose_ is true the state
  // machine skips detection-based PICK targeting entirely: as soon as
  // DETECTING_PICK is entered, pick_target_ is synthesised at world
  // (fixed_pick_x_, fixed_pick_y_, 0) and the cycle proceeds directly
  // through APPROACHING_PICK → DESCEND → GRIP → LIFT → VERIFY → PLACE.
  // Used when the camera calibration is too poor for reliable kitkat
  // detection but the hand-place flow still works. The kitkat YOLO can
  // keep running in the background — it's just ignored for the pick
  // decision.
  bool   use_fixed_pick_pose_;
  double fixed_pick_x_;
  double fixed_pick_y_;
  // Retry cap for fixed-pick mode. The robot's grasp reproducibility
  // isn't perfect at sub-cm levels, so a "fixed" XY may sometimes miss.
  // With retry_on_failed_verify enabled in this mode, the state machine
  // will go back to DETECTING_PICK after a failed verify; this counter
  // caps how many full pick attempts happen before we give up and
  // proceed to the place phase (drops whatever the gripper has, which
  // may be air — but that's preferable to spinning forever).
  int    max_fixed_pick_attempts_;
  int    fixed_pick_attempts_{0};

  // Place target source — controls how place_target_ is populated:
  //   "detection" : wait for a place_class detection in /detections (default)
  //   "fixed"     : use fixed_drop_x/y (equivalent to use_fixed_drop_pose=true)
  //   "tf"        : look up place_tf_frame in the TF tree
  std::string place_mode_;
  std::string place_tf_frame_;      // e.g. "right_wrist_locked" when tf-mode
  double      place_tf_max_age_s_;  // reject stale transforms older than this

  // When place_mode_=="tf", APPROACHING_PLACE / DESCENDING_TO_PLACE /
  // RETRACTING use place_target_.z + these offsets instead of the fixed
  // hover/drop/lift heights (which are meant for a flat table). Kept
  // small so the TCP stays inside the NED2 reach envelope even when the
  // operator's hand is near the workspace edge.
  double      place_tf_hover_offset_m_;
  double      place_tf_drop_offset_m_;
  double      place_tf_lift_offset_m_;
  // Planar (XY) bias applied to the latched place_target_ across all
  // place modes (detection, fixed, tf). Lets the operator nudge the drop
  // point a few cm away from the raw detection / TF / fixed coordinate
  // (e.g. so the bar lands on the palm instead of directly over the wrist
  // joint, or to back the kitkat drop off the table edge). Defaults are
  // 0,0 — no shift. Mirrors pick_offset_x/y on the pick side.
  double      place_offset_x_m_;
  double      place_offset_y_m_;
  bool   loop_mode_;             // after placing, look for more pick targets
  double loop_scene_timeout_s_;  // give up looking for more after this long

  // Multi-object scheduling: when several pick-class objects are visible
  // in one frame we commit to exactly one, rather than EMA-blending them
  // into an empty centroid.
  std::string pick_selection_mode_;  // nearest_to_base | rightmost | leftmost | nearest_to_x_axis
  double      pick_accept_radius_m_; // update EMA only with detections this
                                     // close to the current target; others
                                     // are assumed to be different objects

  // Drop-stack offset — raise approach / descent / retract by this many
  // metres per completed drop so successive objects don't crash into
  // the growing tower.
  double drop_stack_increment_m_;
  int    drops_completed_{0};

  // State
  PnPState state_{PnPState::WAITING_FOR_INIT};
  geometry_msgs::msg::PointStamped pick_target_;
  geometry_msgs::msg::PointStamped pre_verify_pick_target_;  // saved before verification
  geometry_msgs::msg::PointStamped place_target_;
  bool pick_target_valid_{false};
  bool pick_target_initialized_{false};  // true after first detection (EMA needs a seed)
  bool pick_target_latched_{false};      // true once committed — stops updates during descent
  int  pick_detection_count_{0};         // frames seen — must reach min_pick_detections_ before valid
  int  min_pick_detections_;             // per-profile: how many frames to confirm a target
  bool place_target_valid_{false};
  bool place_target_latched_{false};     // true once cow is confirmed — stops updates when arm occludes it
  bool move_group_ready_{false};

  // Grip verification state
  bool verify_moved_clear_{false};       // arm has moved out of camera view
  rclcpp::Time verify_start_time_;       // when verification wait began

  // Loop-mode state
  bool loop_moved_clear_{false};
  rclcpp::Time loop_scan_start_time_;

  // Helper: send gripper command asynchronously
  void send_gripper_async(uint8_t cmd_type);
  void on_gripper_goal_response(GoalHandleTool::SharedPtr goal_handle);
  void on_gripper_result(const GoalHandleTool::WrappedResult & result);

  // Gripper async state
  bool gripper_busy_{false};
  bool gripper_succeeded_{false};
  PnPState gripper_next_state_{PnPState::DONE};
  int grip_attempts_{0};
  int approach_pick_attempts_{0};

  // ROS interfaces
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<ToolAction>::SharedPtr gripper_client_;
};

}  // namespace metis_moveit2_perception
