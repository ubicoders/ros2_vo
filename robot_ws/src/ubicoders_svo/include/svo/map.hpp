#ifndef __MAP_H__
#define __MAP_H__

#include "svo/frame.hpp"
#include "svo/map_point.hpp"

namespace UbiSVO {

class Map {

public:
  using MapPointType = std::unordered_map<u_int32_t, std::shared_ptr<MapPoint>>;
  using KeyFrameType = std::map<u_int32_t, std::shared_ptr<Frame>>;

  // define local window size here as a default value
  Map(int16_t localWindowSize = 10);

  bool addMapPoint(std::shared_ptr<MapPoint> mapPoint);
  bool addKeyframe(std::shared_ptr<Frame> frame);
  bool removeActiveKeyframe(int16_t frameId);
  void cleanMap();

  KeyFrameType &getActiveKeyFrames() { return activeKeyFramePtrs_; }
  KeyFrameType &getKeyFrames() { return keyFramePtrs_; }

  MapPointType &getActiveMapPoints() { return activeMapPointPtrs_; }
  MapPointType &getMapPoints() { return mapPointPtrs_; }

  bool getRequiredViewerUpdated() { return requiredViewerUpdated_; }
  void setRequiredViewerUpdated(bool flag) { requiredViewerUpdated_ = flag; }

  void MapProducerLoop();

private:
  MapPointType mapPointPtrs_;
  MapPointType activeMapPointPtrs_;
  KeyFrameType keyFramePtrs_;
  KeyFrameType activeKeyFramePtrs_;

  int16_t localWindowSize_;

  bool requiredViewerUpdated_ = true;
};

} // namespace UbiSVO

#endif // __MAP_H__
