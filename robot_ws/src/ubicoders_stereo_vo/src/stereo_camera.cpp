#include "ubicoders_stereo_vo/stereo_camera.hpp"

namespace UbicodersSVO {

StereoCamera::StereoCamera() {
  CameraConfig cam_config;
  baseline = cam_config.baseline;
  fx = cam_config.fx;
  fy = cam_config.fy;
  cx = cam_config.cx;
  cy = cam_config.cy;

  K.at<double>(0, 0) = fx;
  K.at<double>(1, 1) = fy;
  K.at<double>(0, 2) = cx;
  K.at<double>(1, 2) = cy;

  P1.at<double>(0, 0) = fx;
  P1.at<double>(1, 1) = fy;
  P1.at<double>(0, 2) = cx;
  P1.at<double>(1, 2) = cy;

  P2.at<double>(0, 0) = fx;
  P2.at<double>(1, 1) = fy;
  P2.at<double>(0, 2) = cx;
  P2.at<double>(1, 2) = cy;
  P2.at<double>(0, 3) = -baseline * fx;
}

StereoCamera::~StereoCamera() {}

} // namespace UbicodersSVO
