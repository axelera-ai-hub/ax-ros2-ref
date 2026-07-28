// (C) Copyright Axelera AI 2026
// All Rights Reserved
//
// Standalone executable entry point for the Metis inference node, used when
// running the node outside a component container.

#include <rclcpp/rclcpp.hpp>
#include "metis_ros2_inference/metis_inference_node.hpp"

// Standalone entry point — runs the node without a component container.
// For zero-copy intra-process use, load via a ComposableNodeContainer instead.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<metis_ros2_inference::MetisInferenceNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
