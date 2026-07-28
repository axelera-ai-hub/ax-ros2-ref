// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Declares MetisInferenceNode, a composable ROS2 node that runs object
// detection on the Axelera Metis AIPU and publishes vision_msgs/Detection2DArray.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

// AxRuntime C API — needed here for axrTensorInfo stored by value
#include <axruntime/axruntime.h>

// OpenCV — needed for preprocess_frame signature
#include <opencv2/core.hpp>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

// ONNX Runtime — forward-declared; full type included in .cpp only.
// Out-of-line destructor ensures unique_ptr cleanup happens where types are complete.
namespace Ort { class Session; class Env; }

namespace metis_ros2_inference
{

class MetisInferenceNode : public rclcpp::Node
{
public:
  explicit MetisInferenceNode(const rclcpp::NodeOptions & options);
  ~MetisInferenceNode();

private:
  // --- Queue payload types ---

  struct FramePacket {
    std::vector<int8_t>   data;    // quantized INT8 input tensor
    std_msgs::msg::Header header;  // propagated from camera (KR 1.5)
  };

  struct ResultPacket {
    std::vector<std::vector<int8_t>> tensors;  // raw INT8 outputs (one per output tensor)
    std_msgs::msg::Header            header;
  };

  // --- Private methods ---
  void                worker_loop();
  void                image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void                process_results();
  std::vector<int8_t> preprocess_frame(const cv::Mat & frame);

  // --- ROS2 interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr        image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
  rclcpp::TimerBase::SharedPtr                                     result_timer_;

  // --- Parameters ---
  std::string          model_dir_;
  std::string          model_name_;
  int                  aipu_cores_{4};
  float                conf_threshold_{0.25f};
  float                nms_threshold_{0.45f};
  std::array<float, 3> mean_  {0.485f, 0.456f, 0.406f};
  std::array<float, 3> stddev_{0.229f, 0.224f, 0.225f};
  bool                 normalize_{false};

  // COCO class labels (optional — loaded from labels_path parameter)
  std::vector<std::string> labels_;

  // --- AxRuntime handles (initialised once in constructor — KR 1.3) ---
  axrContext *       axr_ctx_     {nullptr};
  axrModel *         axr_model_   {nullptr};
  axrConnection *    axr_conn_    {nullptr};
  axrModelInstance * axr_instance_{nullptr};

  // Tensor metadata cached from the model at construction time
  std::vector<axrTensorInfo> input_infos_;
  std::vector<axrTensorInfo> output_infos_;

  // Pre-allocated I/O buffers (sized once at construction)
  std::vector<int8_t>              input_buffer_;
  std::vector<std::vector<int8_t>> output_buffers_;

  // --- ONNX Runtime (postprocess — created once in constructor — KR 1.3) ---
  std::unique_ptr<Ort::Env>     ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  std::vector<std::string>      ort_input_names_;
  std::vector<std::string>      ort_output_names_;

  // --- Worker thread infrastructure ---
  std::thread       worker_thread_;
  std::atomic<bool> worker_running_{false};

  std::queue<FramePacket>  inqueue_;
  std::mutex               inqueue_mtx_;
  std::condition_variable  inqueue_cv_;

  std::queue<ResultPacket> outqueue_;
  std::mutex               outqueue_mtx_;

  // --- Per-frame preprocessing state (written by image_callback, read by process_results) ---
  float                scale_{1.0f};
  int                  x_offset_{0};
  int                  y_offset_{0};
  std_msgs::msg::Header current_header_;
};

}  // namespace metis_ros2_inference
