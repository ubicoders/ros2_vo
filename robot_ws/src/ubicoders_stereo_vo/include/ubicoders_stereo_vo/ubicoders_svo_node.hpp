#ifndef __UBICODERS_SVO_NODE_H__
#define __UBICODERS_SVO_NODE_H__

#include "ubicoders_stereo_vo/ubicoders_svo_publishers.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace UbicodersSVO {

class UbicodersSVO : public rclcpp::Node {
public:
  UbicodersSVO();

  using Image = sensor_msgs::msg::Image;
  using ImageSyncPolicy =
      message_filters::sync_policies::ApproximateTime<Image, Image>;

private:
  void
  imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_image,
                const sensor_msgs::msg::Image::ConstSharedPtr &right_image);

private:
  // Subscribers
  std::shared_ptr<message_filters::Subscriber<Image>> sub_img_left;
  std::shared_ptr<message_filters::Subscriber<Image>> sub_img_right;
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>>
      stereoPairSync;

  // Publishers
};

} // namespace UbicodersSVO

#endif // __UBICODERS_SVO_NODE_H__
