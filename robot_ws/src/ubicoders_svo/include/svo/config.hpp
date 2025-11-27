#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace UbiSVO {

struct CameraConfig {
  double fx = 718.8560;
  double fy = 718.8560;
  double cx = 607.1928;
  double cy = 185.2157;
  double baseline = 0.537;
};

struct ROS2NodeConfig {
  std::string node_name = "stereo_visual_odometry";
  std::string input_img_left_topic = "/stereo/image_left";
  std::string input_img_right_topic = "/stereo/image_right";
  std::string output_img_debug_topic = "/ubicoders/debug_image";
  std::string output_pc_topic = "/ubicoders/point_cloud";
  std::string output_path_topic = "/ubicoders/path";
  std::string output_frame_name = "ubicoders_svo";

  double msg_pub_rate = 30.0;
};

} // namespace UbiSVO

#endif // __CONFIG_H__
