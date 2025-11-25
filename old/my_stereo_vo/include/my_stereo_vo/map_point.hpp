#ifndef __MAP_POINT_H__
#define __MAP_POINT_H__
#include "my_stereo_vo/feature.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <memory>

namespace StereoSLAM {

class Feature; // Forward declaration

/**
 * @brief Represents a 3D landmark in the world.
 *
 * Bidirectional Relationship:
 * - MapPoint needs to know which Features observe it (to optimize its
 * position).
 * - Feature needs to know which MapPoint it corresponds to (to use 3D info).
 *
 * This creates a "Cyclic Dependency". We solve it by:
 * 1. Forward declaring `class Feature;` here.
 * 2. Including `feature.hpp` in `map_point.cpp` (not the header, if possible,
 * but here we need the type for weak_ptr).
 *
 * Purpose:
 * - Stores the 3D position (x, y, z) of a landmark.
 * - Tracks which Features (2D observations) observe this 3D point.
 *
 * Relationship with Feature:
 * - 1 MapPoint is observed by N Features (1:N).
 * - Conversely, 1 Feature observes exactly 1 MapPoint (N:1).
 * - This allows us to link observations from different frames to the same
 * physical point.
 */
class MapPoint {
public:
  using Ptr = std::shared_ptr<MapPoint>;
  /**
   * @brief Default constructor. Generates a unique ID.
   */
  MapPoint();
  /**
   * @brief Constructor with initial world point. Generates a unique ID.
   * @param worldPoint The initial 3D position of the map point.
   */
  MapPoint(const Eigen::Vector3d &worldPoint);

  /**
   * @brief Get the number of Features observing this MapPoint.
   * Used to determine if a point is "good" (seen by many frames) or "bad" (seen
   * by few).
   */
  int16_t getObservationCount() const;

  /**
   * @brief Add a Feature that observes this MapPoint.
   * Called when a new match is found or triangulated.
   */
  bool addObserve(std::shared_ptr<Feature> feature);

  /**
   * @brief Remove a Feature observation.
   * Called when a Frame is removed or the point is deemed invalid.
   */
  bool removeObserve(std::shared_ptr<Feature> feature);

  /**
   * @brief Get the 3D world coordinates of this MapPoint.
   * @return A reference to the Eigen::Vector3d representing the world point.
   */
  Eigen::Vector3d &getWorldPoint() { return worldPoint_; }
  /**
   * @brief Set the 3D world coordinates of this MapPoint.
   * @param point The new 3D position.
   */
  void setWorldPoint(Eigen::Vector3d &point) { worldPoint_ = point; }

  /**
   * @brief Get the list of Features observing this MapPoint.
   * @return A reference to the vector of weak pointers to Features.
   */
  std::vector<std::weak_ptr<Feature>> &getObserve() { return obsFeatures_; };

  /**
   * @brief Unique identifier for this MapPoint.
   */
  const int32_t id;
  /**
   * @brief Flag indicating if this MapPoint is currently being tracked by the
   * local mapping thread. Used to prioritize processing or avoid redundant
   * operations.
   */
  bool isLocalPoint = false;
  bool isOutlier_ = false;

private:
  /**
   * @brief The 3D position of the landmark in world coordinates.
   */
  Eigen::Vector3d worldPoint_;

  /**
   * @brief Cached count of Features observing this MapPoint.
   * Updated when addObserve or removeObserve is called.
   */
  int16_t observationCount_;

  /**
   * @brief List of weak pointers to Features that observe this MapPoint.
   * Weak pointers prevent circular dependencies and allow Features to be
   * deallocated independently.
   */
  std::vector<std::weak_ptr<Feature>> obsFeatures_;

  /**
   * @brief Static counter to generate unique IDs for MapPoints.
   */
  static int32_t nextMapPointId;
};
} // namespace StereoSLAM
#endif // __MAP_POINT_H__