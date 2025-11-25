#ifndef __UBICODERS_SVO_CONFIG_H__
#define __UBICODERS_SVO_CONFIG_H__

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace UbicodersSVO {

struct CameraConfig {
  double fx = 718.8560;
  double fy = 718.8560;
  double cx = 607.1928;
  double cy = 185.2157;
  double baseline = 0.57;

  Eigen::Matrix3d K;
  Eigen::Vector3d t;
  Sophus::SE3d pose;

  CameraConfig() {
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    t << 0, 0, 0;
    pose = Sophus::SE3d();
  }
};

struct ROS2NodeConfig {
  std::string input_img_left_topic = "/stereo/image_left";
  std::string input_img_right_topic = "/stereo/image_right";
  std::string output_img_debug_topic = "/ubicoders/svo/debug_image";
  std::string output_pc_topic = "/ubicoders/svo/point_cloud";
  std::string output_path_topic = "/ubicoders/svo/path";
  std::string output_frame_name = "ubicoders_svo";

  double msg_pub_rate = 30.0;
};

} // namespace UbicodersSVO

#endif // __UBICODERS_SVO_CONFIG_H__
