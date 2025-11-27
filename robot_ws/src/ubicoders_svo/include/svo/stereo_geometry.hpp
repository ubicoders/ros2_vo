#ifndef __PINHOLE_CAMERA_H__
#define __PINHOLE_CAMERA_H__

#include "svo/config.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace UbiSVO {

class StereoGeometry {

public:
  StereoGeometry();
  // getters
  cv::Mat get_K();
  cv::Mat get_T1();
  cv::Mat get_T2();

  // functions
  cv::Point2f pixel2camera(const cv::Point2f &point);
  Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w,
                              const Sophus::SE3d &T_c_w);
  Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w,
                               const Sophus::SE3d &T_c_w);
  Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c);

private:
  CameraConfig scamConfig;
  double fx;
  double fy;
  double cx;
  double cy;
  double baseline;
  cv::Mat K;
  cv::Mat T1; // left extrinsic, 3x4 matrix with identity matrix
  cv::Mat T2; // right extrinsic, 3x4 matrix
  Sophus::SE3d T_l2r;
};
} // namespace UbiSVO

#endif // __PINHOLE_CAMERA_H__