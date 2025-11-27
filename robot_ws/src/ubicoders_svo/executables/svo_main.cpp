#include "svo/svo_node.hpp"

int32_t main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;

  // ros2 node
  auto basic_vo_node = std::make_shared<UbiSVO::StereoVisualOdometry>(options);

  rclcpp::spin(basic_vo_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}