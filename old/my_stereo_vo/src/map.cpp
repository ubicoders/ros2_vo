#include "my_stereo_vo/map.hpp"

namespace StereoSLAM {

Map::Map() {
  // The Map is the "Long-term Memory" of the SLAM system.
  // Without it, the robot is just performing "Visual Odometry" (VO) - knowing
  // how it moved from frame to frame, but forgetting where it started.
  // With a Map, it becomes SLAM - it can recognize places it has been before
  // (Loop Closure) and refine its path (Bundle Adjustment).
}

void Map::addKeyframe(std::shared_ptr<Frame> frame) {
  if (activeKeyFramePtrs_.find(frame->frameId) == activeKeyFramePtrs_.end()) {
    activeKeyFramePtrs_[frame->frameId] = frame;
    std::cout << "Added Keyframe #" << frame->frameId << std::endl;
  }
}

void Map::addMapPoint(MapPoint::Ptr mapPoint) {
  // The Story of "Active" Points:
  // In a large environment, the map can grow to millions of points.
  // We cannot process all of them every single frame (it would be too slow).
  // So, we keep a set of "Active" points - usually the ones the robot can
  // currently see or has seen recently.
  //
  // This function adds a new landmark to this "Working Memory".

  // 1. Check if we already know this point.
  // We use `find(id) == end()` which is the C++ way of asking:
  // "Did I reach the end of my list without finding this ID?"
  if (activeMapPointPtrs_.find(mapPoint->id) == activeMapPointPtrs_.end()) {
    // 2. If it's new, add it to our memory.
    activeMapPointPtrs_[mapPoint->id] = mapPoint;
  } else {
    // If we already have it, we do nothing.
    // This prevents "Double Vision" where the robot thinks one landmark
    // is actually two different ones, which would corrupt the map.
  }
}

} // namespace StereoSLAM