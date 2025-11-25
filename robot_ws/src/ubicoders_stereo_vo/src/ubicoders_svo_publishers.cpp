#include "ubicoders_stereo_vo/ubicoders_svo_publishers.hpp"

namespace UbicodersSVO {

UbicodersSVOPublishers::UbicodersSVOPublishers() {}

void UbicodersSVOPublishers::init(rclcpp::Node *node) {
  ROS2NodeConfig config = UbicodersSVO::ROS2NodeConfig();

  pub_img_debug = node->create_publisher<sensor_msgs::msg::Image>(
      config.output_img_debug_topic, 10);
  pub_pc = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      config.output_pc_topic, 10);
  pub_path =
      node->create_publisher<nav_msgs::msg::Path>(config.output_path_topic, 10);
}

void UbicodersSVOPublishers::publishDebugImage(
    const cv::Mat &image, const std_msgs::msg::Header &header) {
  if (pub_img_debug->get_subscription_count() > 0) {
    cv_bridge::CvImage cv_image(header, "bgr8", image);
    pub_img_debug->publish(*cv_image.toImageMsg());
  }
}

void UbicodersSVOPublishers::publishPointCloud(
    const sensor_msgs::msg::PointCloud2 &pc) {}

void UbicodersSVOPublishers::publishPath(const nav_msgs::msg::Path &path) {}

} // namespace UbicodersSVO
