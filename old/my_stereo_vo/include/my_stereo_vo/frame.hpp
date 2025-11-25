#ifndef __FRAME_H__
#define __FRAME_H__
#include "my_stereo_vo/feature.hpp"
#include "my_stereo_vo/pinhole_camera.hpp"
#include <memory>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <vector>
namespace StereoSLAM {

// Forward declaration of Feature class
class Feature;
/**
 * @brief Frame class (Partial for Step 2)
 *
 * Only holds:
 * - The stereo images.
 * - The list of features.
 * - An ID.
 */
class Frame {
public:
  using Ptr = std::shared_ptr<Frame>;
  Frame();

  const int32_t frameId;
  cv::Mat imageL;
  cv::Mat imageR;

  // Features in the left image
  // Frame owns these features via shared_ptr.
  // Relationship: 1 Frame -> N Features (1:N)
  std::vector<std::shared_ptr<Feature>> featurePtrs;

  // Features in the right image (matched)
  // These correspond 1-to-1 with featurePtrs by index.
  // If a match is found, rightFeaturePtrs[i] is the feature in the right image
  // corresponding to featurePtrs[i] in the left image.
  // If no match is found, the pointer is nullptr.
  std::vector<std::shared_ptr<Feature>> rightFeaturePtrs;

  /**
   * @brief Set this frame as a keyframe.
   * Keyframes are used for Bundle Adjustment and Loop Closure.
   */
  void setKeyFrame();
  /**
   * @brief Check if this frame is a keyframe.
   */
  bool isKeyFrame() const { return isKeyFrame_; }
  // Pose: Transformation from World to Camera (T_wc)
  Sophus::SE3d T_wc;
  std::shared_ptr<PinholeCamera> camera_;

private:
  static int32_t nextFrameId;
  bool isKeyFrame_ = false;
};
} // namespace StereoSLAM
#endif // __FRAME_H__