#ifndef __FEATURE_H__
#define __FEATURE_H__
#include "my_stereo_vo/frame.hpp"
#include "my_stereo_vo/map_point.hpp"
#include <memory>
#include <opencv2/opencv.hpp>
namespace StereoSLAM {
class Frame;
class MapPoint;
/**
 * @brief Feature class (Partial for Step 2)
 *
 * Only holds:
 * - The 2D position (cv::KeyPoint).
 * - A pointer to the Frame it belongs to.
 */
class Feature {

public:
  using Ptr = std::shared_ptr<Feature>;

  // Constructor for a new feature found in the current frame
  Feature(const std::shared_ptr<Frame> frame, const cv::KeyPoint &point);

  // Constructor for a tracked feature (with previous position)
  Feature(const std::shared_ptr<Frame> frame, const cv::KeyPoint &point,
          const cv::Point2f &prevPoint);

  // Previous position (for tracking visualization)
  const cv::Point2f prevPoint = cv::Point2f(0.0, 0.0);
  /**
   * @brief Reference to the Frame this feature belongs to.
   *
   * @brief Reference to the Frame that owns this feature.
   *
   * Relationship: This Feature belongs to exactly 1 Frame.
   * (The overall relationship is N Features to 1 Frame).
   *
   * We use std::weak_ptr here to avoid a cyclic dependency (memory leak).
   * Frame holds std::shared_ptr<Feature>, so Feature must hold
   * std::weak_ptr<Frame>. If Feature held std::shared_ptr<Frame>, the reference
   * counts would never reach zero.
   */
  const std::weak_ptr<Frame> framePtr; // feature has it's parent frame

  // The 2D position of this feature in the image
  const cv::KeyPoint point;

  // True if this feature is from the left image, False if from the right image
  bool isLeftFeature = true;
  bool isInlier = true;

  // Reference to the MapPoint this feature observes
  std::weak_ptr<MapPoint> mapPointPtr; // Added
};
} // namespace StereoSLAM
#endif // __FEATURE_H__