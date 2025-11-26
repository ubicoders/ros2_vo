#include "stereo_visual_slam/pinhole_camera.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace StereoSLAM {

PinholeCamera::PinholeCamera() {
  fx = scamConfig.fx;
  fy = scamConfig.fy;
  cx = scamConfig.cx;
  cy = scamConfig.cy;
  baseline = scamConfig.baseline;
  T_l2r = Sophus::SE3d(Eigen::Matrix3d::Identity(),
                       Eigen::Vector3d(-baseline, 0.0, 0.0));
  K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = fx;
  K.at<double>(1, 1) = fy;
  K.at<double>(0, 2) = cx;
  K.at<double>(0, 2) = cx;
  K.at<double>(1, 2) = cy;

  T1 = (cv::Mat_<float>(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);

  auto pose = T_l2r.matrix();
  T2 = (cv::Mat_<float>(3, 4) << pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3),
        pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3), pose(2, 0), pose(2, 1),
        pose(2, 2), pose(2, 3));
}

cv::Mat PinholeCamera::get_K() { return K; }

cv::Mat PinholeCamera::get_T1() { return T1; }

cv::Mat PinholeCamera::get_T2() { return T2; }

cv::Point2f PinholeCamera::pixel2camera(const cv::Point2f &p_p) {

  double x = (p_p.x - cx) / fx;
  double y = (p_p.y - cy) / fy;

  cv::Point2f point = cv::Point2f(0.0, 0.0);
  point.x = x;
  point.y = y;
  return point;
}

Eigen::Vector2d PinholeCamera::world2pixel(const Eigen::Vector3d &p_w,
                                           const Sophus::SE3d &T_c_w) {
  return camera2pixel(world2camera(p_w, T_c_w));
}

Eigen::Vector3d PinholeCamera::world2camera(const Eigen::Vector3d &p_w,
                                            const Sophus::SE3d &T_c_w) {
  return T_c_w * p_w;
}

Eigen::Vector2d PinholeCamera::camera2pixel(const Eigen::Vector3d &p_c) {
  return Eigen::Vector2d(fx * p_c(0, 0) / p_c(2, 0) + cx,
                         fy * p_c(1, 0) / p_c(2, 0) + cy);
}

} // namespace StereoSLAM