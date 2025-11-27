#ifndef __VIEWER_H__
#define __VIEWER_H__

#include "svo/config.hpp"
#include "svo/frame.hpp"
#include "svo/map.hpp"
#include "svo/stereo_geometry.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace UbiSVO {
class Viewer {
public:
  Viewer(
      std::shared_ptr<Map> map, std::shared_ptr<StereoGeometry> stereoCam,
      rclcpp::Clock::SharedPtr clock,
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub,
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub,
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub);
  void update();
  void debugImageUpdate(const std::shared_ptr<Frame> frame);

private:
  void mapPointUpdate();
  void pathUpdate();

private:
  ROS2NodeConfig ros2NodeConfig;
  std::shared_ptr<Map> map_;
  std::shared_ptr<StereoGeometry> stereoCam_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;

  sensor_msgs::msg::PointCloud2 pointCloudMsg_;

  // Track last printed keyframe ID to avoid duplicate prints
  uint32_t lastPrintedFrameId_ = 0;
};
} // namespace UbiSVO

#endif // __VIEWER_H__