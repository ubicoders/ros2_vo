#ifndef __PINHOLE_CAMERA_H__
#define __PINHOLE_CAMERA_H__
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
namespace StereoSLAM {
class PinholeCamera {
public:
  using Ptr = std::shared_ptr<PinholeCamera>;

  PinholeCamera(double fx, double fy, double cx, double cy, double baseline,
                Sophus::SE3d pose);
  // Coordinate Transforms

  // p_p: Point in Pixel coordinates (u, v)
  // p_c: Point in Camera coordinates (x, y, z)
  // p_w: Point in World coordinates (x, y, z)
  // T_c_w: Transformation from World to Camera

  Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p, double depth = 1);
  cv::Point2f pixel2camera(const cv::Point2f &p_p);
  Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w,
                               const Sophus::SE3d &T_c_w);
  Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c);
  Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w,
                              const Sophus::SE3d &T_c_w);

  // Triangulation: Calculate 3D point from Stereo Disparity
  Eigen::Vector3d pixel2World(const cv::Point2f &pointL,
                              const cv::Point2f &pointR);
  double fx_, fy_, cx_, cy_, baseline_;
  Sophus::SE3d pose_;
  Sophus::SE3d pose_inv_;
};
} // namespace StereoSLAM
#endif // __PINHOLE_CAMERA_H__