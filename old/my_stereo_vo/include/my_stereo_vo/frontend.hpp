#ifndef __FRONTEND_H__
#define __FRONTEND_H__
#include "my_stereo_vo/backend.hpp"
#include "my_stereo_vo/frame.hpp"
#include "my_stereo_vo/map.hpp"
#include "my_stereo_vo/pinhole_camera.hpp"
#include <memory>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace StereoSLAM {
enum class Status { INIT, TRACKING, LOSS };

class Frontend {
public:
  typedef std::shared_ptr<Frontend> Ptr; // Added

  // step 1
  Frontend();
  //  step 2
  Frontend(std::shared_ptr<PinholeCamera> camera);
  // step 4
  Frontend(std::shared_ptr<PinholeCamera> camera, std::shared_ptr<Map> map);

  // Set the map
  void setMap(std::shared_ptr<Map> map) { map_ = map; } // Added

  // Set the backend
  void setBackend(std::shared_ptr<Backend> backend) {
    backend_ = backend;
  } // Added

  // Set the camera
  void setCamera(std::shared_ptr<PinholeCamera> camera) {
    camera_ = camera;
  } // Added

  /**
   * @brief Main processing step for a new frame.
   *
   * This function acts as the main entry point for the VO pipeline for each
   * incoming frame. It handles the state machine logic (Initialization ->
   * Tracking -> Lost).
   *
   * Better names might be:
   * - processFrame(): Clearly indicates processing a single frame.
   * - addFrame(): Suggests adding data to the pipeline.
   * - track(): Emphasizes the tracking aspect of VO.
   *
   * @param frame The current frame containing stereo images.
   * @return true if processing was successful.
   */
  bool step(std::shared_ptr<Frame> frame);

private:
  /**
   * @brief Initialize the map with the first frame.
   */
  void init();
  /**
   * @brief Track features from the previous frame.
   */
  void tracking();
  /**
   * @brief Track features from previous frame to current frame using Optical
   * Flow.
   * @return Number of features successfully tracked.
   */
  int16_t trackingFeature();
  /**
   * @brief Estimate camera pose using tracked features.
   * @return Number of inlier features after pose estimation.
   */
  int16_t estimatePose();
  /**
   * @brief Update MapPoint observations with current frame features.
   */
  void updateObservation();
  int16_t createLeftFeature();
  /**
   * @brief Find corresponding features in the right image using LK Optical
   * Flow.
   * @return Number of features found in right image.
   */
  int16_t matchInRight();

  /**
   * @brief Triangulate 3D points from stereo matches and add to Map.
   */
  void createMapPoint();

private:
  std::shared_ptr<Frame> currentFrame_ = nullptr;
  std::shared_ptr<PinholeCamera> camera_ = nullptr; // Added
  std::shared_ptr<Map> map_ = nullptr;              // Added
  std::shared_ptr<Backend> backend_ = nullptr;
  Status status_ = Status::INIT;
  std::shared_ptr<Frame> prevFrame_ = nullptr;

  cv::Ptr<cv::GFTTDetector> gftt_;
};
} // namespace StereoSLAM
#endif // __FRONTEND_H__