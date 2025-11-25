#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ubicoders_svo_node");
  RCLCPP_INFO(node->get_logger(), "Hello from ubicoders_svo_node!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
