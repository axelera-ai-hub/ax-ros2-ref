// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Implementation of LiveInferenceNode. Displays a live camera feed with
// detection and color overlays for demos.

#include "axros_helper_nodes/live_inference_node.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace axros_helper_nodes
{

LiveInferenceNode::LiveInferenceNode(const rclcpp::NodeOptions & options)
: Node("live_inference", options)
{
  // ── Declare + get parameters ────────────────────────────────────────────────
  declare_parameter<std::string>("camera_device",   "/dev/video0");
  declare_parameter<std::string>("output_topic",    "image_raw");
  declare_parameter<double>     ("fps",             30.0);
  declare_parameter<std::string>("frame_id",        "camera");
  declare_parameter<int>        ("summary_every_s", 10);
  declare_parameter<std::string>("window_name",     "AxROS Live Inference");
  // Resolution + pixel-format controls. The original node hardcoded
  // 1920x1080 in the default pixel format (YUYV on most UVC cameras),
  // which fails to open a second camera that only supports MJPG at e.g.
  // 1280x720@60fps. Leave fourcc empty to skip setting it.
  declare_parameter<int>        ("frame_width",     1920);
  declare_parameter<int>        ("frame_height",    1080);
  declare_parameter<std::string>("fourcc",          "");
  // When false, skip the in-node display window + the camera_frame_annotated
  // republisher. On slower hosts (e.g. Raspberry Pi 5) the per-frame
  // toCvCopy + resize + imshow + republish in the same single-threaded
  // executor starves the 30 Hz capture timer and drops /image_raw to ~8 Hz.
  declare_parameter<bool>       ("enable_display",  true);

  camera_device_   = get_parameter("camera_device").as_string();
  output_topic_    = get_parameter("output_topic").as_string();
  fps_             = get_parameter("fps").as_double();
  frame_id_        = get_parameter("frame_id").as_string();
  summary_every_s_ = get_parameter("summary_every_s").as_int();
  window_name_     = get_parameter("window_name").as_string();
  const int         frame_width    = get_parameter("frame_width").as_int();
  const int         frame_height   = get_parameter("frame_height").as_int();
  const std::string fourcc         = get_parameter("fourcc").as_string();
  const bool        enable_display = get_parameter("enable_display").as_bool();

  // ── Open camera ─────────────────────────────────────────────────────────────
  cap_.open(camera_device_, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    throw std::runtime_error("LiveInferenceNode: failed to open camera device: " +
      camera_device_);
  }
  // FOURCC must be set BEFORE width/height/fps — OpenCV applies pixel-format
  // first, then negotiates resolution against whatever that format supports.
  if (fourcc.size() == 4) {
    cap_.set(cv::CAP_PROP_FOURCC,
      cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]));
  }
  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  frame_width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
  cap_.set(cv::CAP_PROP_FPS,          fps_);

  RCLCPP_INFO(get_logger(),
    "LiveInferenceNode:\n"
    "  camera_device:   %s\n"
    "  output_topic:    %s\n"
    "  frame:           %dx%d @ %.1f fps%s%s\n"
    "  frame_id:        %s\n"
    "  window_name:     %s\n"
    "  summary_every_s: %d",
    camera_device_.c_str(),
    output_topic_.c_str(),
    frame_width, frame_height, fps_,
    fourcc.empty() ? "" : " fourcc=", fourcc.c_str(),
    frame_id_.c_str(),
    window_name_.c_str(),
    summary_every_s_);

  // ── Publishers ───────────────────────────────────────────────────────────────
  image_pub_    = create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

  // ── Detection cache subscription (cheap, always on) ──────────────────────────
  // det_cache only stores latest detections — used by the display path and by
  // the periodic [Summary] log. Cheap enough to leave on regardless.
  latest_dets_time_ = this->now();
  det_cache_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "detections", 10,
    std::bind(&LiveInferenceNode::det_cache_callback, this, std::placeholders::_1));

  // ── Timers ───────────────────────────────────────────────────────────────────
  capture_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / fps_),
    std::bind(&LiveInferenceNode::capture_callback, this));

  summary_timer_ = create_wall_timer(
    std::chrono::seconds(summary_every_s_),
    std::bind(&LiveInferenceNode::summary_callback, this));

  if (!enable_display) {
    RCLCPP_INFO(get_logger(),
      "enable_display=false — skipping window, camera_frame_annotated republisher, "
      "and color-detection overlay subscription (capture only).");
    display_available_ = false;
    return;
  }

  annotated_pub_ = create_publisher<sensor_msgs::msg::Image>("camera_frame_annotated", 10);

  display_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "image_raw", 10,
    std::bind(&LiveInferenceNode::display_image_callback, this, std::placeholders::_1));

  // ── Color detection subscriber (for yellow_detector overlay) ─────────────────
  color_det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "/color_detections", 10,
    [this](vision_msgs::msg::Detection2DArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(color_det_mtx_);
      latest_color_dets_ = msg;
    });

  // ── Pick ROI polygon subscriber (red_detector publishes once, latched) ──────
  // Use TRANSIENT_LOCAL + KEEP_LAST(1) to match the publisher's QoS so we
  // receive the polygon even if we subscribe after it was sent. Best-effort
  // would silently drop the latched message on connect.
  rclcpp::QoS roi_qos(1);
  roi_qos.transient_local();
  pick_roi_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/pick_roi_polygon", roi_qos,
    [this](geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(pick_roi_mtx_);
      latest_pick_roi_ = msg;
    });

  // ── Display guard ────────────────────────────────────────────────────────────
  const char * x11     = std::getenv("DISPLAY");
  const char * wayland = std::getenv("WAYLAND_DISPLAY");
  if (!x11 && !wayland) {
    RCLCPP_WARN(get_logger(),
      "No display detected (DISPLAY/WAYLAND_DISPLAY not set). "
      "Annotated frames will be published but not shown in a window.");
    display_available_ = false;
  } else {
    try {
      cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      display_available_ = true;
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(get_logger(),
        "cv::namedWindow failed: %s. Disabling live display.", e.what());
      display_available_ = false;
    }
  }
}

LiveInferenceNode::~LiveInferenceNode()
{
  cv::destroyAllWindows();
  if (cap_.isOpened()) {
    cap_.release();
  }
  RCLCPP_INFO(get_logger(), "LiveInferenceNode shutting down");
}

// ── capture_callback ──────────────────────────────────────────────────────────

void LiveInferenceNode::capture_callback()
{
  cv::Mat frame;
  if (!cap_.read(frame)) {
    RCLCPP_WARN(get_logger(), "Failed to capture frame from %s", camera_device_.c_str());
    return;
  }

  std_msgs::msg::Header header;
  header.stamp    = this->now();
  header.frame_id = frame_id_;
  auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

  image_pub_->publish(*msg);
  ++frame_count_;
}

// ── display_image_callback ───────────────────────────────────────────────────
// Fires on every image, not every synced (image, detection) pair. Displays
// cached detections if they're fresh, otherwise just shows the raw frame.

void LiveInferenceNode::display_image_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Grab fresh detections from the cache. If too old, don't overlay them —
  // we don't want ghost boxes lingering after the object has left the scene.
  vision_msgs::msg::Detection2DArray::ConstSharedPtr dets;
  {
    std::lock_guard<std::mutex> lock(latest_dets_mtx_);
    if (latest_dets_) {
      double age = (this->now() - latest_dets_time_).seconds();
      if (age <= DET_OVERLAY_STALE_S) {
        dets = latest_dets_;
      }
    }
  }

  cv::Mat annotated = dets
    ? draw_detections(cv_ptr->image, *dets)
    : cv_ptr->image.clone();

  // ── Overlay pick ROI polygon (world-frame 60×40 cm KitKat zone) ─────────────
  // The red_detector projects the ROI rectangle into pixel coordinates and
  // latches it on /pick_roi_polygon. Drawn under the crosshair so a target
  // inside the ROI still draws cleanly on top of the polygon edge.
  {
    std::lock_guard<std::mutex> lock(pick_roi_mtx_);
    if (latest_pick_roi_ && latest_pick_roi_->polygon.points.size() >= 2) {
      std::vector<cv::Point> pts;
      pts.reserve(latest_pick_roi_->polygon.points.size());
      for (const auto & p : latest_pick_roi_->polygon.points) {
        pts.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
      }
      const cv::Scalar roi_color(0, 255, 255);  // yellow (BGR)
      cv::polylines(annotated, std::vector<std::vector<cv::Point>>{pts},
        /*isClosed*/ true, roi_color, /*thickness*/ 2, cv::LINE_AA);
      cv::putText(annotated, "ROI",
        {pts.front().x + 6, std::max(0, pts.front().y - 8)},
        cv::FONT_HERSHEY_SIMPLEX, 0.6, roi_color, 2, cv::LINE_AA);
    }
  }

  // ── Overlay color detections (crosshair) ────────────────────────────────────
  {
    std::lock_guard<std::mutex> lock(color_det_mtx_);
    if (latest_color_dets_) {
      for (const auto & det : latest_color_dets_->detections) {
        if (det.results.empty()) continue;
        const auto & hyp = det.results[0].hypothesis;
        int cx = static_cast<int>(det.bbox.center.position.x);
        int cy = static_cast<int>(det.bbox.center.position.y);
        int sz = 30;
        cv::Scalar color(0, 0, 255);
        cv::line(annotated, {cx - sz, cy}, {cx + sz, cy}, color, 2);
        cv::line(annotated, {cx, cy - sz}, {cx, cy + sz}, color, 2);
        cv::circle(annotated, {cx, cy}, 6, color, -1);
        cv::putText(annotated, hyp.class_id,
          {cx + 20, cy - 15}, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv::LINE_AA);
      }
    }
  }

  // ── Display ──────────────────────────────────────────────────────────────────
  if (display_available_) {
    try {
      cv::Mat display_frame;
      cv::resize(annotated, display_frame,
        cv::Size(annotated.cols / 2, annotated.rows / 2),
        0, 0, cv::INTER_LINEAR);
      cv::imshow(window_name_, display_frame);
      cv::waitKey(1);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(get_logger(),
        "cv::imshow failed: %s. Disabling live display.", e.what());
      display_available_ = false;
    }
  }

  // ── Publish annotated frame ───────────────────────────────────────────────────
  auto out_msg = cv_bridge::CvImage(img_msg->header, "bgr8", annotated).toImageMsg();
  annotated_pub_->publish(*out_msg);
}

// ── det_cache_callback ───────────────────────────────────────────────────────
// Just caches the latest detections + stamps them for staleness checking,
// and feeds the 10-second summary counters.

void LiveInferenceNode::det_cache_callback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & det_msg)
{
  {
    std::lock_guard<std::mutex> lock(latest_dets_mtx_);
    latest_dets_      = det_msg;
    latest_dets_time_ = this->now();
  }
  {
    std::lock_guard<std::mutex> lock(counts_mtx_);
    for (const auto & det : det_msg->detections) {
      if (!det.results.empty()) {
        detection_counts_[det.results[0].hypothesis.class_id]++;
      }
    }
  }
}

// ── summary_callback ──────────────────────────────────────────────────────────

void LiveInferenceNode::summary_callback()
{
  std::lock_guard<std::mutex> lock(counts_mtx_);

  if (detection_counts_.empty()) {
    RCLCPP_INFO(get_logger(), "[Summary] No detections in last %ds", summary_every_s_);
  } else {
    std::vector<std::pair<std::string, int>> sorted(
      detection_counts_.begin(), detection_counts_.end());
    std::sort(sorted.begin(), sorted.end(),
      [](const auto & a, const auto & b) { return a.second > b.second; });

    std::ostringstream ss;
    ss << "[Summary] Last " << summary_every_s_ << "s detections: ";
    for (size_t i = 0; i < sorted.size(); ++i) {
      if (i > 0) { ss << ", "; }
      ss << sorted[i].first << " x" << sorted[i].second;
    }
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
  }

  detection_counts_.clear();
}

// ── draw_detections ───────────────────────────────────────────────────────────

cv::Mat LiveInferenceNode::draw_detections(
  const cv::Mat & frame,
  const vision_msgs::msg::Detection2DArray & detections)
{
  cv::Mat out = frame.clone();

  for (const auto & det : detections.detections) {
    if (det.results.empty()) {
      continue;
    }

    const auto & hyp = det.results[0].hypothesis;
    float score      = static_cast<float>(hyp.score);
    std::string label = hyp.class_id;

    cv::Scalar color;
    if (score >= 0.7f) {
      color = cv::Scalar(0, 255, 0);    // green  — high confidence
    } else if (score >= 0.4f) {
      color = cv::Scalar(0, 165, 255);  // orange — medium confidence
    } else {
      color = cv::Scalar(0, 0, 255);    // red    — low confidence
    }

    int x1 = static_cast<int>(det.bbox.center.position.x - det.bbox.size_x / 2.0);
    int y1 = static_cast<int>(det.bbox.center.position.y - det.bbox.size_y / 2.0);
    int x2 = static_cast<int>(det.bbox.center.position.x + det.bbox.size_x / 2.0);
    int y2 = static_cast<int>(det.bbox.center.position.y + det.bbox.size_y / 2.0);

    x1 = std::clamp(x1, 0, out.cols - 1);
    y1 = std::clamp(y1, 0, out.rows - 1);
    x2 = std::clamp(x2, 0, out.cols - 1);
    y2 = std::clamp(y2, 0, out.rows - 1);

    cv::rectangle(out, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);

    std::ostringstream label_ss;
    label_ss << label << " " << std::fixed << std::setprecision(2) << score;
    std::string label_text = label_ss.str();

    int baseline = 0;
    cv::Size text_size = cv::getTextSize(
      label_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

    cv::rectangle(out,
      cv::Point(x1, y1 - text_size.height - 6),
      cv::Point(x1 + text_size.width, y1),
      color, cv::FILLED);

    cv::putText(out, label_text,
      cv::Point(x1, y1 - 4),
      cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

    int cx = (x1 + x2) / 2;
    int cy = (y1 + y2) / 2;
    cv::circle(out, cv::Point(cx, cy), 3, color, cv::FILLED);
  }

  std::string count_text = "Detections: " + std::to_string(detections.detections.size());
  cv::putText(out, count_text, cv::Point(10, 30),
    cv::FONT_HERSHEY_SIMPLEX, 0.8,
    cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

  return out;
}

}  // namespace axros_helper_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(axros_helper_nodes::LiveInferenceNode)
