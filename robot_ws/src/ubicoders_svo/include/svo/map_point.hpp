#ifndef __MAP_POINT_H__
#define __MAP_POINT_H__

#include "svo/feature.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <memory>

namespace UbiSVO {

class Feature;

class MapPoint {

public:
  using Ptr = std::shared_ptr<MapPoint>;

  MapPoint() : id(++nextMapPointId), observationCount(0) {}

  MapPoint(Eigen::Vector3d &worldPoint)
      : id(++nextMapPointId), worldPoint_(worldPoint), observationCount(0) {}

  int16_t getObservationCount() const { return observationCount; }

  // add the feature to the map point, this mappoint is in the active frame.
  bool addObserve(std::shared_ptr<Feature> feature) {
    ++observationCount;
    obsFeatures_.push_back(feature);
    return true;
  }

  bool removeObserve(std::shared_ptr<Feature> feature) {
    --observationCount;
    // remove_if: shifts all valid elements to the front and returns an iterator
    // to the new end of the vector. [A, X, B, X, C, X] -> [A, B, C, X, X, X]
    // new_end points to the first invalid element.
    auto new_end = std::remove_if(
        obsFeatures_.begin(), obsFeatures_.end(),
        [&feature](const std::weak_ptr<Feature> &wf) {
          auto sp = wf.lock();        // sp = shared_ptr
          return sp && sp == feature; // if shared ptr exist and it is the same
                                      // as the feature, return true
        });
    bool found = (new_end != obsFeatures_.end());
    obsFeatures_.erase(new_end, obsFeatures_.end());
    return found;
  }

  Eigen::Vector3d &getWorldPoint() { return worldPoint_; }

  void setWorldPoint(Eigen::Vector3d &point) { worldPoint_ = point; }

  std::vector<std::weak_ptr<Feature>> &getObserve() { return obsFeatures_; };

  const int32_t id;

  // isLocalPoint: true if this map point is part of the active local map
  // (observed by any active keyframe in the sliding window).
  bool isLocalPoint = false;

private:
  Eigen::Vector3d worldPoint_;
  int16_t observationCount;
  std::vector<std::weak_ptr<Feature>> obsFeatures_;
  inline static int32_t nextMapPointId = 0;
};
} // namespace UbiSVO

#endif // __MAP_POINT_H__