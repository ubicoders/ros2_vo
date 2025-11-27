#include "svo/map.hpp"
#include <cstddef>

namespace UbiSVO {
Map::Map(int16_t localWindowSize) : localWindowSize_(localWindowSize) {}

bool Map::addMapPoint(std::shared_ptr<MapPoint> mapPoint) {
  if (mapPointPtrs_.find(mapPoint->id) == mapPointPtrs_.end()) {
    mapPointPtrs_[mapPoint->id] = mapPoint;
    activeMapPointPtrs_[mapPoint->id] = mapPoint;

    setRequiredViewerUpdated(true);
    return true;
  }

  return false;
}

bool Map::addKeyframe(std::shared_ptr<Frame> frame) {
  if (keyFramePtrs_.find(frame->getFrameId()) == keyFramePtrs_.end()) {
    keyFramePtrs_[frame->getFrameId()] = frame;
    activeKeyFramePtrs_[frame->getFrameId()] = frame;

    // sliding window
    if ((size_t)localWindowSize_ < activeKeyFramePtrs_.size()) {
      removeActiveKeyframe(activeKeyFramePtrs_.begin()->first);
    }

    setRequiredViewerUpdated(true);
    return true;
  }

  return false;
}

bool Map::removeActiveKeyframe(int16_t frameId) {
  auto it = activeKeyFramePtrs_.find(frameId);
  if (it != activeKeyFramePtrs_.end()) {
    std::cout << "Remove Active Frame: " << frameId << std::endl;
    auto &frame = activeKeyFramePtrs_[frameId];
    for (auto &feature : frame->leftFeaturePtrs) {
      if (!feature->mapPointPtr.expired()) {
        auto mapPoint = feature->mapPointPtr.lock();
        mapPoint->removeObserve(feature);
      }
    }
    activeKeyFramePtrs_.erase(it);
    cleanMap();
    return true;
  }
  return false;
}

void Map::cleanMap() {
  for (auto iter = activeMapPointPtrs_.begin();
       iter != activeMapPointPtrs_.end();) {
    if (iter->second->getObservationCount() == 0) {
      iter->second->isLocalPoint = false;
      iter = activeMapPointPtrs_.erase(iter);
    } else {
      ++iter;
    }
  }
}

} // namespace UbiSVO