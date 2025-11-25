#ifndef __MAP_H__
#define __MAP_H__
#include "my_stereo_vo/map_point.hpp"
#include <unordered_map>
namespace StereoSLAM {

/**
 * @brief The global map containing all active landmarks.
 *
 * Purpose:
 * - Acts as a database for all 3D points (MapPoints) and KeyFrames.
 * - Manages the "Active" set of points used for tracking and optimization.
 * - In Step 4, we use it mainly to store triangulated points for visualization.
 */
class Map {
public:
  using KeyFrameType = std::map<uint32_t, std::shared_ptr<Frame>>;
  using Ptr = std::shared_ptr<Map>;
  using MapPointType = std::unordered_map<uint32_t, MapPoint::Ptr>;
  Map();

  /**
   * @brief Add a new MapPoint to the map.
   * Called by Frontend when a new point is triangulated.
   */
  void addMapPoint(MapPoint::Ptr mapPoint);

  /**
   * @brief Add a keyframe to the map.
   */
  void addKeyframe(std::shared_ptr<Frame> frame);

  KeyFrameType &getActiveKeyFrames() { return activeKeyFramePtrs_; }

  /**
   * @brief Get all active MapPoints.
   * "Active" points are those currently relevant for tracking (e.g., in the
   * local window). In Step 4, this is just all points we've added.
   */
  MapPointType &getActiveMapPoints() { return activeMapPointPtrs_; }

private:
  KeyFrameType activeKeyFramePtrs_;

  // Storage for active MapPoints, indexed by ID
  // "Active" MapPoints are the ones currently being used for:
  // 1. Tracking: Matching with the current frame.
  // 2. Optimization: Being adjusted by the Local Bundle Adjustment.
  //
  // In a full SLAM system, old points that are far away or not visible
  // are moved to a "Inactive" list (or just kept in the global map)
  // to save computational resources.
  //
  // For Step 4, we treat all points as active, but we use this name
  // to prepare for the Local Window optimization in later steps.
  MapPointType activeMapPointPtrs_;
};
} // namespace StereoSLAM
#endif // __MAP_H__