#ifndef _STEREO_VISUAL_ODOMETRY_H_
#define _STEREO_VISUAL_ODOMETRY_H_

#include "rclcpp/rclcpp.hpp"
#include "svo/backend.hpp"
#include "svo/config.hpp"
#include "svo/frame.hpp"
#include "svo/frontend.hpp"
#include "svo/map.hpp"
#include "svo/stereo_geometry.hpp"
#include "svo/viewer.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace UbiSVO {

class StereoVisualOdometry : public rclcpp::Node {

public:
  using Image = sensor_msgs::msg::Image;
  using ImageSyncPolicy =
      message_filters::sync_policies::ApproximateTime<Image, Image>;
  explicit StereoVisualOdometry(const rclcpp::NodeOptions &);

private:
  void ImageCallback(const Image::ConstSharedPtr &leftImage,
                     const Image::ConstSharedPtr &rightImage);

private:
  ROS2NodeConfig ros2NodeConfig;
  std::shared_ptr<message_filters::Subscriber<Image>> leftImageSub_;
  std::shared_ptr<message_filters::Subscriber<Image>> rightImageSub_;
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>> syncStereo_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;

  // VO components
  std::unique_ptr<Viewer> viewer;
  std::shared_ptr<Frontend> frontend;
  std::shared_ptr<Backend> backend;
  std::shared_ptr<Map> map;
  std::shared_ptr<StereoGeometry> stereoCam;
};
} // namespace UbiSVO

#endif // _STEREO_VISUAL_ODOMETRY_H_