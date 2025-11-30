#ifndef __FRAME_H__
#define __FRAME_H__

#include "svo/feature.hpp"
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

// Forward declaration of Feature class
class Feature;

namespace UbiSVO {

class Frame {

public:
  Frame() : frameId(++nextFrameId) {}

  int32_t getFrameId() { return frameId; }
  void setKeyFrame() { isKeyFrame = true; }

  cv::Mat imageL;
  cv::Mat imageR;
  std::vector<std::shared_ptr<Feature>> leftFeaturePtrs;
  std::vector<std::shared_ptr<Feature>> rightFeaturePtrs;

  Sophus::SE3d T_w2c; // world to camera: (0, 0, 0) camera position to world
                      // position, i.e Cam pose in World coordinate

  bool isKeyFrame = false;
  bool needUpdate = false;

private:
  const int32_t frameId;
  inline static int32_t nextFrameId = 0;
};
} // namespace UbiSVO

#endif // __FRAME_H__