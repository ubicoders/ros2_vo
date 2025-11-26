#ifndef __STEREO_CAMERA_HPP__
#define __STEREO_CAMERA_HPP__
#include "ubicoders_stereo_vo/config.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <opencv2/opencv.hpp>

namespace UbicodersSVO {

class StereoCamera {

public:
  StereoCamera();
  ~StereoCamera();

  cv::Point2f pixel2camera(const cv::Point2f &point);

private:
  float baseline;
  float fx, fy, cx, cy;
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat P1 = cv::Mat_<float>(3, 4);
  cv::Mat P2 = cv::Mat_<float>(3, 4);
};

} // namespace UbicodersSVO

#endif // __STEREO_CAMERA_HPP__
