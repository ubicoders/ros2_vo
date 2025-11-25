
#ifndef __STEREO_VISUAL_SLAM_H__
#define __STEREO_VISUAL_SLAM_H__
#include "my_stereo_vo/frame.hpp" // Added
#include "my_stereo_vo/frontend.hpp"
#include "my_stereo_vo/map.hpp" // Added
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <my_stereo_vo/pinhole_camera.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp> // Added for visualization

namespace StereoSLAM {
class StereoVisualSLAM : public rclcpp::Node {
public:
  using Image = sensor_msgs::msg::Image;

  /**
   * @brief ImageSyncPolicy defines the synchronization strategy for stereo
   * images.
   *
   * We use `ApproximateTime` instead of `ExactTime` because in real-world
   * scenarios, the timestamps of the left and right images might not be
   * identical down to the nanosecond. This can happen due to:
   * - Slight delays in camera triggering.
   * - Network transmission latencies.
   * - Driver implementation details.
   *
   * ApproximateTime allows for a small tolerance (epsilon) when matching
   * messages.
   */
  using ImageSyncPolicy =
      message_filters::sync_policies::ApproximateTime<Image, Image>;

  explicit StereoVisualSLAM(const rclcpp::NodeOptions &);

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
  nav_msgs::msg::Path pathMsg_;
  /**
   * @brief Callback function for synchronized stereo images.
   *
   * This function is called ONLY when a pair of left and right images
   * are successfully synchronized by the `syncStereo_` synchronizer.
   *
   * @param leftImage The image from the left camera.
   * @param rightImage The image from the right camera.
   */
  void ImageCallback(const Image::ConstSharedPtr &leftImage,
                     const Image::ConstSharedPtr &rightImage);

private:
  std::shared_ptr<Frontend> frontend_;
  std::shared_ptr<PinholeCamera> stereoCam_;
  std::shared_ptr<Map> map_;
  std::shared_ptr<Backend> backend_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub_;

private:
  // Subscribers wrapped in message_filters to allow synchronization.
  // Standard rclcpp::Subscription cannot be easily synchronized.
  std::shared_ptr<message_filters::Subscriber<Image>> leftImageSub_;
  std::shared_ptr<message_filters::Subscriber<Image>> rightImageSub_;

  // The synchronizer object that manages the queue and matching logic.
  // It takes the policy (ImageSyncPolicy) and the subscribers as input.
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>> syncStereo_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub_;
};
} // namespace StereoSLAM
#endif // __STEREO_VISUAL_SLAM_H__