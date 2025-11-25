#include "my_stereo_vo/feature.hpp"
namespace StereoSLAM {
Feature::Feature(const std::shared_ptr<Frame> frame, const cv::KeyPoint &point)
    : framePtr(frame), point(point) {}

Feature::Feature(const std::shared_ptr<Frame> frame, const cv::KeyPoint &point,
                 const cv::Point2f &prevPoint)
    : framePtr(frame), point(point), prevPoint(prevPoint) {}
} // namespace StereoSLAM