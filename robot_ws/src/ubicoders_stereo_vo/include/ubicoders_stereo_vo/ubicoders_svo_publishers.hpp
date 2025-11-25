#ifndef __UBICODERS_SVO_PUBLISHERS_H__
#define __UBICODERS_SVO_PUBLISHERS_H__

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ubicoders_stereo_vo/config.hpp>

namespace UbicodersSVO {

// singleton class to publish messages
class UbicodersSVOPublishers {
public:
  static UbicodersSVOPublishers &getInstance() {
    static UbicodersSVOPublishers instance;
    return instance;
  }

  void init(rclcpp::Node *node);

  void publishDebugImage(const cv::Mat &image,
                         const std_msgs::msg::Header &header);
  void publishPointCloud(const sensor_msgs::msg::PointCloud2 &pc);
  void publishPath(const nav_msgs::msg::Path &path);

private:
  UbicodersSVOPublishers();

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_debug;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
};

} // namespace UbicodersSVO

#endif // __UBICODERS_SVO_PUBLISHERS_H__
