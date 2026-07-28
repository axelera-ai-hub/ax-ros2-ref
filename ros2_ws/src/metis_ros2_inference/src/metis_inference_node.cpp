// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Implementation of the Metis AIPU inference node. Converts incoming camera
// frames, runs object detection on the Metis AIPU via the AxRuntime C API,
// executes the ONNX postprocess graph, and publishes Detection2DArray.

#include "metis_ros2_inference/metis_inference_node.hpp"

// AxRuntime C++ helpers (axr::ptr, axr::to_object)
#include <axruntime/axruntime.hpp>

// ONNX Runtime C++ API (full types — forward-declared in header)
#include <onnxruntime_cxx_api.h>

// OpenCV
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

// cv_bridge
#include <cv_bridge/cv_bridge.hpp>

// vision_msgs sub-types
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <cstring>
#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <chrono>
#include <thread>

namespace metis_ros2_inference
{

MetisInferenceNode::MetisInferenceNode(const rclcpp::NodeOptions & options)
: Node("metis_inference_node", options)
{
  // Declare parameters (KR 1.4: no hardcoded paths)
  declare_parameter<std::string>("model_dir",  "");
  declare_parameter<std::string>("model_name", "");
  declare_parameter<int>        ("aipu_cores",  4);
  declare_parameter<double>     ("conf_threshold", 0.25);
  declare_parameter<double>     ("nms_threshold",  0.45);
  // normalize / mean / stddev are RESERVED parameters. They are parsed and
  // stored but NOT yet applied: preprocess_frame() currently implements only
  // YOLOv5 style scaling (raw /255 via scale=1/255, zp=-128). They exist so a
  // future mean/stddev normalization path (e.g. YOLOX) can be added without
  // changing the public parameter interface.
  declare_parameter<std::vector<double>>("mean",   {0.485, 0.456, 0.406});
  declare_parameter<std::vector<double>>("stddev", {0.229, 0.224, 0.225});
  declare_parameter<std::string>("input_topic",  "image_raw");
  declare_parameter<std::string>("output_topic", "detections");
  declare_parameter<bool>       ("normalize", false);  // reserved, see note above
  declare_parameter<std::string>("labels_path", "");
  // Retry knobs for axr_device_connect() / axr_load_model_instance().
  // The Metis kernel driver occasionally fails the initial DMA buffer
  // transfer ("DMABUF_METIS_XFER failed: Input/output error",
  // 0x70010001) — usually transient. Retrying after a short sleep
  // succeeds in the vast majority of these cases.
  declare_parameter<int>        ("aipu_load_retries",        3);
  declare_parameter<double>     ("aipu_load_retry_delay_s",  1.5);

  const int    aipu_load_retries       = get_parameter("aipu_load_retries").as_int();
  const double aipu_load_retry_delay_s = get_parameter("aipu_load_retry_delay_s").as_double();

  model_dir_  = get_parameter("model_dir").as_string();
  model_name_ = get_parameter("model_name").as_string();
  aipu_cores_ = get_parameter("aipu_cores").as_int();
  conf_threshold_ = static_cast<float>(get_parameter("conf_threshold").as_double());
  nms_threshold_  = static_cast<float>(get_parameter("nms_threshold").as_double());
  normalize_ = get_parameter("normalize").as_bool();

  const auto mean_v   = get_parameter("mean").as_double_array();
  const auto stddev_v = get_parameter("stddev").as_double_array();
  for (int i = 0; i < 3; ++i) {
    mean_[i]   = static_cast<float>(mean_v[i]);
    stddev_[i] = static_cast<float>(stddev_v[i]);
  }

  const auto input_topic  = get_parameter("input_topic").as_string();
  const auto output_topic = get_parameter("output_topic").as_string();

  // Validate required parameters
  if (model_dir_.empty()) {
    throw std::runtime_error("Parameter 'model_dir' must not be empty");
  }
  if (model_name_.empty()) {
    throw std::runtime_error("Parameter 'model_name' must not be empty");
  }

  // Optional: load class labels from file
  const auto labels_path = get_parameter("labels_path").as_string();
  if (!labels_path.empty()) {
    std::ifstream f(labels_path);
    if (!f.is_open()) {
      RCLCPP_WARN(get_logger(), "Could not open labels_path: %s", labels_path.c_str());
    } else {
      std::string line;
      while (std::getline(f, line)) {
        if (!line.empty()) { labels_.push_back(line); }
      }
      RCLCPP_INFO(get_logger(), "  labels        : %zu classes from %s",
        labels_.size(), labels_path.c_str());
    }
  }

  // AxRuntime init (KR 1.3: created once in constructor)
  axr_ctx_ = axr_create_context();
  if (!axr_ctx_) {
    throw std::runtime_error("axr_create_context() failed");
  }

  const std::string model_json_path = model_dir_ + "/model.json";
  axr_model_ = axr_load_model(axr_ctx_, model_json_path.c_str());
  if (!axr_model_) {
    throw std::runtime_error(
      "axr_load_model() failed for '" + model_json_path + "' — " +
      axr_last_error_string(AXR_OBJECT(axr_ctx_)));
  }

  // Cache tensor metadata
  const size_t num_inputs  = axr_num_model_inputs(axr_model_);
  const size_t num_outputs = axr_num_model_outputs(axr_model_);
  input_infos_.resize(num_inputs);
  output_infos_.resize(num_outputs);
  for (size_t i = 0; i < num_inputs;  ++i) { input_infos_[i]  = axr_get_model_input (axr_model_, i); }
  for (size_t i = 0; i < num_outputs; ++i) { output_infos_[i] = axr_get_model_output(axr_model_, i); }

  // Connect to device (nullptr = first available, aipu_cores_ sub-devices).
  // Retry up to aipu_load_retries times — the kernel driver occasionally
  // fails the initial connect when DMA buffers from a previous process
  // haven't been fully torn down yet. Sleeping then retrying lets the
  // driver settle without requiring the user to relaunch.
  {
    const int max_attempts = std::max(1, aipu_load_retries + 1);
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
      axr_conn_ = axr_device_connect(
        axr_ctx_, nullptr, static_cast<size_t>(aipu_cores_), nullptr);
      if (axr_conn_) break;
      if (attempt == max_attempts) {
        throw std::runtime_error(
          std::string("axr_device_connect() failed after ") +
          std::to_string(attempt) + " attempts — " +
          axr_last_error_string(AXR_OBJECT(axr_ctx_)));
      }
      RCLCPP_WARN(get_logger(),
        "axr_device_connect() failed (attempt %d/%d) — sleeping %.1fs and retrying. "
        "Last error: %s",
        attempt, max_attempts, aipu_load_retry_delay_s,
        axr_last_error_string(AXR_OBJECT(axr_ctx_)));
      std::this_thread::sleep_for(
        std::chrono::duration<double>(aipu_load_retry_delay_s));
    }
  }

  // Load model instance onto device — same retry pattern. This is where
  // the DMABUF transfer happens, so this is the call that most often
  // throws "DMABUF_METIS_XFER failed" / "Level-zero memory operation
  // failed: 0x70010001". The instance pointer is null on failure so we
  // don't need to release anything between attempts.
  {
    const int max_attempts = std::max(1, aipu_load_retries + 1);
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
      axr_instance_ = axr_load_model_instance(axr_conn_, axr_model_, nullptr);
      if (axr_instance_) break;
      if (attempt == max_attempts) {
        throw std::runtime_error(
          std::string("axr_load_model_instance() failed after ") +
          std::to_string(attempt) + " attempts — " +
          axr_last_error_string(AXR_OBJECT(axr_ctx_)));
      }
      RCLCPP_WARN(get_logger(),
        "axr_load_model_instance() failed (attempt %d/%d) — sleeping %.1fs and retrying. "
        "Last error: %s",
        attempt, max_attempts, aipu_load_retry_delay_s,
        axr_last_error_string(AXR_OBJECT(axr_ctx_)));
      std::this_thread::sleep_for(
        std::chrono::duration<double>(aipu_load_retry_delay_s));
    }
  }

  // Pre-allocate I/O buffers based on tensor sizes from the model
  if (!input_infos_.empty()) {
    input_buffer_.resize(axr_tensor_size(&input_infos_[0]));
  }
  output_buffers_.resize(num_outputs);
  for (size_t i = 0; i < num_outputs; ++i) {
    output_buffers_[i].resize(axr_tensor_size(&output_infos_[i]));
  }

  // ONNX Runtime init (KR 1.3: created once in constructor)
  ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "metis_postprocess");

  Ort::SessionOptions session_opts;
  session_opts.SetIntraOpNumThreads(1);

  const std::string onnx_path = model_dir_ + "/postprocess_graph.onnx";
  ort_session_ = std::make_unique<Ort::Session>(*ort_env_, onnx_path.c_str(), session_opts);

  // Cache input/output names so we don't allocate on every frame
  Ort::AllocatorWithDefaultOptions allocator;
  for (size_t i = 0; i < ort_session_->GetInputCount(); ++i) {
    ort_input_names_.push_back(
      std::string(ort_session_->GetInputNameAllocated(i, allocator).get()));
  }
  for (size_t i = 0; i < ort_session_->GetOutputCount(); ++i) {
    ort_output_names_.push_back(
      std::string(ort_session_->GetOutputNameAllocated(i, allocator).get()));
  }

  // ROS2 interfaces
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic, 10,
    std::bind(&MetisInferenceNode::image_callback, this, std::placeholders::_1));

  detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(output_topic, 10);

  // Worker thread + result timer
  worker_running_ = true;
  worker_thread_  = std::thread(&MetisInferenceNode::worker_loop, this);

  result_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&MetisInferenceNode::process_results, this));

  // Startup log
  RCLCPP_INFO(get_logger(), "MetisInferenceNode ready");
  RCLCPP_INFO(get_logger(), "  model_name    : %s", model_name_.c_str());
  RCLCPP_INFO(get_logger(), "  model_dir     : %s", model_dir_.c_str());
  RCLCPP_INFO(get_logger(), "  aipu_cores    : %d", aipu_cores_);
  RCLCPP_INFO(get_logger(), "  conf_threshold: %.2f", conf_threshold_);
  RCLCPP_INFO(get_logger(), "  nms_threshold : %.2f", nms_threshold_);
  RCLCPP_INFO(get_logger(), "  input_topic   : %s", input_topic.c_str());
  RCLCPP_INFO(get_logger(), "  output_topic  : %s", output_topic.c_str());

  for (size_t i = 0; i < input_infos_.size(); ++i) {
    std::ostringstream ss;
    ss << "[";
    for (size_t d = 0; d < input_infos_[i].ndims; ++d) {
      ss << input_infos_[i].dims[d];
      if (d + 1 < input_infos_[i].ndims) { ss << ", "; }
    }
    ss << "]";
    RCLCPP_INFO(get_logger(), "  input[%zu]  shape: %s  scale=%.6f  zp=%d",
      i, ss.str().c_str(), input_infos_[i].scale, input_infos_[i].zero_point);
  }
  for (size_t i = 0; i < output_infos_.size(); ++i) {
    std::ostringstream ss;
    ss << "[";
    for (size_t d = 0; d < output_infos_[i].ndims; ++d) {
      ss << output_infos_[i].dims[d];
      if (d + 1 < output_infos_[i].ndims) { ss << ", "; }
    }
    ss << "]";
    RCLCPP_INFO(get_logger(), "  output[%zu] shape: %s  scale=%.6f  zp=%d",
      i, ss.str().c_str(), output_infos_[i].scale, output_infos_[i].zero_point);
  }
  RCLCPP_INFO(get_logger(), "  ort inputs    : %zu  ort outputs: %zu",
    ort_input_names_.size(), ort_output_names_.size());
}

MetisInferenceNode::~MetisInferenceNode()
{
  // Stop worker thread gracefully before destroying AxRuntime handles
  worker_running_ = false;
  inqueue_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  // Destroy AxRuntime context — this also destroys all child objects
  // (model, connection, instance) so we don't need to call axr_destroy on each
  if (axr_ctx_) {
    axr_destroy(AXR_OBJECT(axr_ctx_));
    axr_ctx_ = nullptr;
  }
}

// preprocess_frame — letterbox resize + INT8 quantize into padded NHWC buffer
// Note: normalize_ flag reserved for models requiring mean/stddev normalization
// (e.g. YOLOX). YOLOv5 uses torch-totensor (raw /255), handled by scale=1/255 zp=-128.
std::vector<int8_t> MetisInferenceNode::preprocess_frame(const cv::Mat & frame)
{
  const auto & info = input_infos_[0];

  // Unpadded model spatial dimensions (e.g. 640×640 from [4,644,656,4])
  const int model_H = static_cast<int>(info.dims[1] - info.padding[1][0] - info.padding[1][1]);
  const int model_W = static_cast<int>(info.dims[2] - info.padding[2][0] - info.padding[2][1]);

  // Letterbox scale (preserve aspect ratio, no upscaling needed for typical cameras)
  scale_ = std::min(
    static_cast<float>(model_W) / static_cast<float>(frame.cols),
    static_cast<float>(model_H) / static_cast<float>(frame.rows));

  const int resized_w = static_cast<int>(std::round(frame.cols * scale_));
  const int resized_h = static_cast<int>(std::round(frame.rows * scale_));
  x_offset_ = (model_W - resized_w) / 2;
  y_offset_ = (model_H - resized_h) / 2;

  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(resized_w, resized_h), 0, 0, cv::INTER_LINEAR);

  // Letterbox canvas — fill value 114 (YOLOv5 standard grey, not 0)
  cv::Mat canvas(model_H, model_W, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(canvas(cv::Rect(x_offset_, y_offset_, resized_w, resized_h)));

  // BGR to RGB
  cv::Mat rgb;
  cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);

  // Fill padded NHWC buffer [N, padded_H, padded_W, padded_C]
  // All slots initialized to zero_point; only the spatial+channel region gets real data.
  const size_t padded_N = info.dims[0];
  const size_t padded_H = info.dims[1];
  const size_t padded_W = info.dims[2];
  const size_t padded_C = info.dims[3];

  std::vector<int8_t> buf(padded_N * padded_H * padded_W * padded_C,
    static_cast<int8_t>(info.zero_point));

  const size_t ch_offset = info.padding[3][0];  // channel start padding (usually 0)

  for (size_t n = 0; n < padded_N; ++n) {
    for (int h = 0; h < model_H; ++h) {
      const size_t dh = static_cast<size_t>(h) + info.padding[1][0];
      for (int w = 0; w < model_W; ++w) {
        const size_t dw   = static_cast<size_t>(w) + info.padding[2][0];
        const size_t base = ((n * padded_H + dh) * padded_W + dw) * padded_C + ch_offset;
        const auto & px   = rgb.at<cv::Vec3b>(h, w);
        // Quantize: int8 = uint8 - 128  (scale=1/255, zp=-128 → int8 = px/scale + zp)
        buf[base + 0] = static_cast<int8_t>(static_cast<int>(px[0]) - 128);
        buf[base + 1] = static_cast<int8_t>(static_cast<int>(px[1]) - 128);
        buf[base + 2] = static_cast<int8_t>(static_cast<int>(px[2]) - 128);
        // buf[base + 3] stays at zero_point — hardware-required 4th channel padding
      }
    }
  }

  return buf;
}

// image_callback — convert, preprocess, non-blocking push to inqueue
void MetisInferenceNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat frame;
  try {
    frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  FramePacket pkt;
  pkt.header = msg->header;  // KR 1.5: propagate camera timestamp
  pkt.data   = preprocess_frame(frame);

  {
    std::lock_guard<std::mutex> lk(inqueue_mtx_);
    if (inqueue_.size() >= 2) {
      inqueue_.pop();
      RCLCPP_DEBUG(get_logger(), "dropped stale frame from inqueue");
    }
    inqueue_.push(std::move(pkt));
  }
  inqueue_cv_.notify_one();
}

// worker_loop — pops from inqueue, runs hardware inference, pushes to outqueue
// Runs on a dedicated thread — axr_run_model_instance must NOT block the executor.
void MetisInferenceNode::worker_loop()
{
  // Build axrArgument arrays once — pointers into pre-allocated member buffers
  // remain valid for the lifetime of the node.
  std::vector<axrArgument> input_args(input_infos_.size());
  std::vector<axrArgument> output_args(output_infos_.size());

  for (size_t i = 0; i < input_infos_.size(); ++i) {
    input_args[i] = {input_buffer_.data(), 0, 0, input_buffer_.size()};
  }
  for (size_t i = 0; i < output_infos_.size(); ++i) {
    output_args[i] = {output_buffers_[i].data(), 0, 0, output_buffers_[i].size()};
  }

  while (worker_running_) {
    std::unique_lock<std::mutex> lk(inqueue_mtx_);
    inqueue_cv_.wait(lk, [this] { return !inqueue_.empty() || !worker_running_; });

    if (!worker_running_ && inqueue_.empty()) { break; }

    auto pkt = std::move(inqueue_.front());
    inqueue_.pop();
    lk.unlock();

    // Copy preprocessed frame into the pre-allocated input buffer
    std::memcpy(input_buffer_.data(), pkt.data.data(), pkt.data.size());

    // Run hardware inference (blocking — isolated to this worker thread)
    const axrResult result = axr_run_model_instance(
      axr_instance_,
      input_args.data(), input_args.size(),
      output_args.data(), output_args.size());

    if (result != AXR_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "axr_run_model_instance failed: %s",
        axr_last_error_string(AXR_OBJECT(axr_ctx_)));
      continue;
    }

    // Copy output buffers into a ResultPacket and push to outqueue
    ResultPacket rpkt;
    rpkt.header = pkt.header;
    rpkt.tensors.resize(output_buffers_.size());
    for (size_t i = 0; i < output_buffers_.size(); ++i) {
      rpkt.tensors[i] = output_buffers_[i];
    }

    {
      std::lock_guard<std::mutex> out_lk(outqueue_mtx_);
      if (outqueue_.size() >= 4) { outqueue_.pop(); }
      outqueue_.push(std::move(rpkt));
    }
  }
}

// process_results — 20ms timer: dequantize → ORT → NMS → publish
void MetisInferenceNode::process_results()
{
  ResultPacket rpkt;
  {
    std::lock_guard<std::mutex> lk(outqueue_mtx_);
    if (outqueue_.empty()) { return; }
    rpkt = std::move(outqueue_.front());
    outqueue_.pop();
  }

  // --- Dequantize + transpose NHWC → NCHW for each output tensor ---
  const size_t num_tensors = rpkt.tensors.size();
  std::vector<std::vector<float>>       ort_inputs(num_tensors);
  std::vector<std::array<int64_t, 4>>   ort_shapes(num_tensors);

  for (size_t ti = 0; ti < num_tensors; ++ti) {
    const auto & info = output_infos_[ti];

    const size_t H = info.dims[1] - info.padding[1][0] - info.padding[1][1];
    const size_t W = info.dims[2] - info.padding[2][0] - info.padding[2][1];
    const size_t C = info.dims[3] - info.padding[3][0] - info.padding[3][1];

    ort_inputs[ti].resize(C * H * W);
    ort_shapes[ti] = {1,
      static_cast<int64_t>(C),
      static_cast<int64_t>(H),
      static_cast<int64_t>(W)};

    const float dq_scale = static_cast<float>(info.scale);
    const int   dq_zp    = info.zero_point;

    for (size_t h = 0; h < H; ++h) {
      const size_t sh = h + info.padding[1][0];
      for (size_t w = 0; w < W; ++w) {
        const size_t sw = w + info.padding[2][0];
        for (size_t c = 0; c < C; ++c) {
          const size_t sc  = c + info.padding[3][0];

          const size_t src = ((0 * info.dims[1] + sh) * info.dims[2] + sw) * info.dims[3] + sc;

          const size_t dst = (c * H + h) * W + w;
          ort_inputs[ti][dst] =
            (static_cast<int>(rpkt.tensors[ti][src]) - dq_zp) * dq_scale;
        }
      }
    }
  }

  // --- Create ORT input tensors ---
  Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::vector<Ort::Value> ort_input_tensors;
  ort_input_tensors.reserve(num_tensors);
  for (size_t ti = 0; ti < num_tensors; ++ti) {
    ort_input_tensors.push_back(Ort::Value::CreateTensor<float>(
      mem,
      ort_inputs[ti].data(), ort_inputs[ti].size(),
      ort_shapes[ti].data(), 4));
  }

  // Build name pointer arrays (ORT API requires const char**)
  std::vector<const char *> in_names, out_names;
  for (const auto & n : ort_input_names_)  { in_names.push_back(n.c_str()); }
  for (const auto & n : ort_output_names_) { out_names.push_back(n.c_str()); }

  std::vector<Ort::Value> ort_outputs;
  try {
    Ort::RunOptions run_opts;
    ort_outputs = ort_session_->Run(
      run_opts,
      in_names.data(),  ort_input_tensors.data(), in_names.size(),
      out_names.data(), out_names.size());
  } catch (const Ort::Exception & e) {
    RCLCPP_ERROR(get_logger(), "ORT session Run() failed: %s", e.what());
    return;
  }

  // --- Parse ONNX postprocess output ---
  // Two layouts supported:
  //   YOLOv5  → [1, N, 85]  (cx, cy, w, h, obj, 80 × class_scores) needs NMS
  //   YOLOv10 → [1, N, 6]   (x1, y1, x2, y2, score, class_id)      already-NMS'd
  // Branching on the trailing dim keeps the rest of the publish pipeline
  // identical: we end up with parallel boxes/scores/class_ids + indices.
  const auto out_shape = ort_outputs[0].GetTensorTypeAndShapeInfo().GetShape();
  if (out_shape.size() < 3 || (out_shape[2] != 85 && out_shape[2] != 6)) {
    RCLCPP_ERROR(get_logger(),
      "Unexpected ORT output shape (expected [1,N,85] for YOLOv5 or "
      "[1,N,6] for YOLOv10)");
    return;
  }

  const int64_t num_preds = out_shape[1];
  const int     stride    = static_cast<int>(out_shape[2]);
  const float * pred_data = ort_outputs[0].GetTensorData<float>();

  std::vector<cv::Rect2d> boxes;
  std::vector<float>      scores;
  std::vector<int>        class_ids;
  std::vector<int>        indices;

  if (stride == 85) {
    // YOLOv5: dense grid output, raw class scores, no NMS yet.
    for (int64_t j = 0; j < num_preds; ++j) {
      const float * pred       = pred_data + j * 85;
      const float   objectness = pred[4];

      int   best_cls   = 0;
      float best_score = 0.0f;
      for (int c = 0; c < 80; ++c) {
        const float s = pred[5 + c] * objectness;
        if (s > best_score) { best_score = s; best_cls = c; }
      }

      if (best_score < conf_threshold_) { continue; }

      const float cx = pred[0], cy = pred[1], bw = pred[2], bh = pred[3];
      boxes.push_back(cv::Rect2d(cx - bw / 2.0, cy - bh / 2.0, bw, bh));
      scores.push_back(best_score);
      class_ids.push_back(best_cls);
    }
    if (!boxes.empty()) {
      cv::dnn::NMSBoxes(boxes, scores, conf_threshold_, nms_threshold_, indices);
    }
  } else {
    // YOLOv10: ultralytics post-NMS layout — first num_preds rows are valid
    // but padded out to a fixed length (300). Confidence-zero rows are the
    // padding; reject them with the same conf_threshold the user configured.
    for (int64_t j = 0; j < num_preds; ++j) {
      const float * pred  = pred_data + j * 6;
      const float   score = pred[4];
      if (score < conf_threshold_) { continue; }

      const float x1 = pred[0], y1 = pred[1], x2 = pred[2], y2 = pred[3];
      boxes.push_back(cv::Rect2d(x1, y1, x2 - x1, y2 - y1));
      scores.push_back(score);
      class_ids.push_back(static_cast<int>(pred[5]));
    }
    // Indices = identity (no NMS to run — model already did it).
    indices.resize(boxes.size());
    std::iota(indices.begin(), indices.end(), 0);
  }

  // --- Build and publish Detection2DArray (KR 1.2 + KR 1.5) ---
  vision_msgs::msg::Detection2DArray det_array;
  det_array.header = rpkt.header;  // propagated camera timestamp — NOT this->now()

  for (const int idx : indices) {
    // Remap from model space (640×640) back to original frame coordinates
    const double x1 = (boxes[idx].x                    - x_offset_) / scale_;
    const double y1 = (boxes[idx].y                    - y_offset_) / scale_;
    const double x2 = (boxes[idx].x + boxes[idx].width  - x_offset_) / scale_;
    const double y2 = (boxes[idx].y + boxes[idx].height - y_offset_) / scale_;

    vision_msgs::msg::Detection2D det;
    det.header                 = rpkt.header;
    det.bbox.center.position.x = (x1 + x2) / 2.0;
    det.bbox.center.position.y = (y1 + y2) / 2.0;
    det.bbox.size_x            = std::abs(x2 - x1);
    det.bbox.size_y            = std::abs(y2 - y1);

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id =
      (class_ids[idx] < static_cast<int>(labels_.size()))
      ? labels_[class_ids[idx]]
      : std::to_string(class_ids[idx]);
    hyp.hypothesis.score = scores[idx];

    det.results.push_back(hyp);
    det_array.detections.push_back(det);
  }

  detection_pub_->publish(det_array);

  if (!indices.empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "Published %zu detection(s)", indices.size());
  }
}

}  // namespace metis_ros2_inference

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(metis_ros2_inference::MetisInferenceNode)
