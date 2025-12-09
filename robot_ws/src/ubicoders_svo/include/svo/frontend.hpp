#ifndef __FRONTEND_H__
#define __FRONTEND_H__

#include "svo/backend.hpp"
#include "svo/frame.hpp"
#include "svo/map.hpp"
#include "svo/map_point.hpp"
#include "svo/stereo_geometry.hpp"
#include <Eigen/Dense>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
namespace UbiSVO {

enum class Status { INIT, TRACKING, LOSS };
class Frontend {
public:
  Frontend(std::shared_ptr<StereoGeometry> stereoCam, std::shared_ptr<Map> map,
           std::shared_ptr<Backend> backend);
  bool step(std::shared_ptr<Frame> frame);

private:
  void tracking();
  void init();
  int16_t createLeftFeature();
  int16_t matchInRight();
  int16_t trackingFeature();
  void create3DMapPoint();
  void updateObservation();
  // void createFbow();
  int16_t estimatePose();

private:
  Status status_ = Status::INIT;
  std::shared_ptr<Frame> currentFrame = nullptr;
  std::shared_ptr<Frame> prevFrame = nullptr;
  std::shared_ptr<StereoGeometry> stereoCam;
  std::shared_ptr<Map> map;
  std::shared_ptr<Backend> backend;

  cv::Ptr<cv::GFTTDetector> gftt;
  cv::Ptr<cv::ORB> orb;

  bool use_backend = true;

  // Low-pass filter variables
  cv::Mat rVecFiltered;
  cv::Mat tVecFiltered;
  bool isFilterInitialized = false;
  const double filterAlpha = 0.9;
};
} // namespace UbiSVO

#endif // __FRONTEND_H__