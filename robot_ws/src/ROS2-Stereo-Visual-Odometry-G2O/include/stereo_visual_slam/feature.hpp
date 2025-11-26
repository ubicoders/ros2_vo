#ifndef __FEATURE_H__
#define __FEATURE_H__

#include "stereo_visual_slam/frame.hpp"
#include "stereo_visual_slam/map_point.hpp"
#include <opencv2/opencv.hpp>

namespace StereoSLAM {

// Forward declaration of Frame class
class Frame;
// Forward declaration of MapPoint class
class MapPoint;

class Feature {

public:
  // type def for shared pointer
  using Ptr = std::shared_ptr<Feature>;

  Feature(const std::shared_ptr<Frame> frame, const cv::KeyPoint &point)
      : framePtr(frame), point(point) {}

  const std::weak_ptr<Frame> framePtr;
  const cv::KeyPoint point;

  std::weak_ptr<MapPoint> mapPointPtr;
  bool isInlier = true;
  bool isLeftFeature = false;
};
} // namespace StereoSLAM

#endif // __FEATURE_H__