#include "ubicoders_stereo_vo/ubicoders_svo_node.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace UbicodersSVO {

UbicodersSVO::UbicodersSVO() : Node("ubicoders_svo_node") {
  UbicodersSVOPublishers::getInstance().init(this);

  ROS2NodeConfig config = ROS2NodeConfig();
  sub_img_left = std::make_shared<message_filters::Subscriber<Image>>(
      this, config.input_img_left_topic);
  sub_img_right = std::make_shared<message_filters::Subscriber<Image>>(
      this, config.input_img_right_topic);

  stereoPairSync =
      std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(
          ImageSyncPolicy(10), *sub_img_left, *sub_img_right);
  stereoPairSync->registerCallback(std::bind(&UbicodersSVO::imageCallback, this,
                                             std::placeholders::_1,
                                             std::placeholders::_2));
}

void UbicodersSVO::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &leftImage,
    const sensor_msgs::msg::Image::ConstSharedPtr &rightImage) {

  cv::Mat img_gray_left =
      cv_bridge::toCvCopy(leftImage, leftImage->encoding)->image;
  cv::Mat img_gray_right =
      cv_bridge::toCvCopy(rightImage, rightImage->encoding)->image;

  // Publish debug image (original input)
  UbicodersSVOPublishers::getInstance().publishDebugImage(img_gray_left,
                                                          leftImage->header);

  // make sure to gray scale the images
  cv::cvtColor(img_gray_left, img_gray_left, cv::COLOR_BGR2GRAY);
  cv::cvtColor(img_gray_right, img_gray_right, cv::COLOR_BGR2GRAY);
}

} // namespace UbicodersSVO

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UbicodersSVO::UbicodersSVO>();
  RCLCPP_INFO(node->get_logger(), "ubicoders_svo_node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
