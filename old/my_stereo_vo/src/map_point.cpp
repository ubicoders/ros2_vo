#include "my_stereo_vo/map_point.hpp"
namespace StereoSLAM {

int32_t MapPoint::nextMapPointId = 0;

MapPoint::MapPoint(const Eigen::Vector3d &worldPoint)
    : id(nextMapPointId++), worldPoint_(worldPoint) {}

MapPoint::MapPoint() : id(nextMapPointId++), observationCount_(0) {}

int16_t MapPoint::getObservationCount() const { return observationCount_; }
bool MapPoint::addObserve(std::shared_ptr<Feature> feature) {
  obsFeatures_.push_back(feature);
  observationCount_++;
  return true;
}

bool MapPoint::removeObserve(std::shared_ptr<Feature> feature) {
  // Implementation for removing observations
  observationCount_--;
  return true;
}
} // namespace StereoSLAM