#include "my_stereo_vo/frontend.hpp"
namespace StereoSLAM {

Frontend::Frontend() {
  // Initialize GFTT Detector: max 150 features, quality 0.01, min dist 20px
  gftt_ = cv::GFTTDetector::create(150, 0.01, 20);
}

// step 2 added
Frontend::Frontend(std::shared_ptr<PinholeCamera> camera) : camera_(camera) {
  gftt_ = cv::GFTTDetector::create(150, 0.01, 20);
}

// step 4 added
Frontend::Frontend(std::shared_ptr<PinholeCamera> camera,
                   std::shared_ptr<Map> map)
    : camera_(camera), map_(map) {
  gftt_ = cv::GFTTDetector::create(150, 0.01, 20);
}

bool Frontend::step(std::shared_ptr<Frame> frame) {
  currentFrame_ = frame;
  currentFrame_->camera_ = camera_;

  if (status_ == Status::INIT) {
    init();
  } else {
    tracking();
  }
  prevFrame_ = currentFrame_;
  return true;
}
void Frontend::init() {
  // Initialize pose to identity (origin)
  currentFrame_->T_wc = Sophus::SE3d();
  // Detect features, match, and triangulate
  createLeftFeature();
  matchInRight();
  createMapPoint();
  // Mark as keyframe and add to map
  currentFrame_->setKeyFrame();
  map_->addKeyframe(currentFrame_);
  std::cout << "Frontend: INIT" << std::endl;
  status_ = Status::TRACKING;
}

void Frontend::tracking() {
  // 1. Track features from previous frame
  int trackingCount = trackingFeature();

  // 2. Estimate camera pose
  int inlierCount = estimatePose();

  std::cout << "Tracking: " << trackingCount << " / Inliers: " << inlierCount
            << std::endl;

  // 3. If we lose too many features, insert a new keyframe
  if (inlierCount <= 70) {
    std::cout << "Set Keyframe #" << currentFrame_->frameId << std::endl;

    updateObservation();
    currentFrame_->setKeyFrame();
    map_->addKeyframe(currentFrame_);

    // Trigger Backend Optimization
    if (backend_) {
      backend_->optimize();
    }

    // Detect new features to replenish the map
    createLeftFeature();
    matchInRight();
    createMapPoint();
  }
}

int16_t Frontend::trackingFeature() {
  if (prevFrame_ == nullptr) {
    status_ = Status::INIT;
    return -1;
  }
  std::vector<cv::Point2f> prevKeypoints, currKeypoints;
  // Prepare inputs: previous feature positions
  for (auto &feature : prevFrame_->featurePtrs) {
    prevKeypoints.push_back(feature->point.pt);
    currKeypoints.push_back(feature->point.pt); // Initial guess
  }
  std::vector<uchar> status;
  cv::Mat error;
  // Run Optical Flow from previous frame to current frame
  cv::calcOpticalFlowPyrLK(
      prevFrame_->imageL, currentFrame_->imageL, prevKeypoints, currKeypoints,
      status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);
  // Create features in current frame for successfully tracked points
  for (size_t i = 0; i < status.size(); ++i) {
    const auto &prevFeature = prevFrame_->featurePtrs[i];
    if (status[i] && !prevFeature->mapPointPtr.expired()) {
      // Create a new feature with the tracked position
      Feature::Ptr feat(
          new Feature(currentFrame_,
                      cv::KeyPoint(currKeypoints[i].x, currKeypoints[i].y, 1.0),
                      prevKeypoints[i]));
      // Link to the same MapPoint
      feat->mapPointPtr = prevFeature->mapPointPtr;
      currentFrame_->featurePtrs.push_back(std::move(feat));
    }
  }
  return currentFrame_->featurePtrs.size();
}
int16_t Frontend::estimatePose() {
  if (prevFrame_ == nullptr) {
    currentFrame_->T_wc = Sophus::SE3d();
    return currentFrame_->featurePtrs.size();
  }
  // 1. Collect 3D-2D correspondences
  // Initialize with previous frame's pose (Constant Position Model)
  // This ensures that if PnP fails, we don't reset to Identity (0,0,0)
  currentFrame_->T_wc = prevFrame_->T_wc;

  // CONCEPT: Pose Estimation (World-to-Current)
  // We use the MapPoints (which are in fixed World Coordinates) as the 3D
  // anchor. We use the current frame's pixels (tracked from previous frame) as
  // the 2D observation. SolvePnP calculates the pose of the Current Frame
  // relative to the World.
  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> imagePoints;
  std::vector<std::shared_ptr<Feature>> validFeatures;
  for (auto &feat : currentFrame_->featurePtrs) {
    if (!feat->mapPointPtr.expired()) {
      auto mapPoint = feat->mapPointPtr.lock();
      auto &worldPos = mapPoint->getWorldPoint();
      objectPoints.push_back(
          cv::Point3f(worldPos.x(), worldPos.y(), worldPos.z()));
      imagePoints.push_back(feat->point.pt);
      validFeatures.push_back(feat);
    }
  }
  if (objectPoints.size() < 10) {
    return 0; // Lost tracking
  }
  // 2. Solve PnP (Perspective-n-Point)
  cv::Mat K = (cv::Mat_<double>(3, 3) << camera_->fx_, 0, camera_->cx_, 0,
               camera_->fy_, camera_->cy_, 0, 0, 1);
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
  cv::Mat rvec, tvec;
  std::vector<int> inliers;

  // Use previous frame's pose as initial guess
  // T_cw_guess = T_wc_prev.inverse()
  Sophus::SE3d T_cw_guess = prevFrame_->T_wc.inverse();
  Eigen::Matrix3d R = T_cw_guess.rotationMatrix();
  Eigen::Vector3d t = T_cw_guess.translation();

  cv::Mat R_cv = (cv::Mat_<double>(3, 3) << R(0, 0), R(0, 1), R(0, 2), R(1, 0),
                  R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));
  cv::Rodrigues(R_cv, rvec);
  tvec = (cv::Mat_<double>(3, 1) << t(0), t(1), t(2));

  // Use Extrinsic Guess (true) and tighter threshold (2.0 pixels)
  bool success = cv::solvePnPRansac(objectPoints, imagePoints, K, distCoeffs,
                                    rvec, tvec, false, 100, 2.0, 0.99, inliers);
  if (success) {
    // 3. Update Pose
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d R_eigen;
    R_eigen << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    Eigen::Vector3d t_eigen(tvec.at<double>(0), tvec.at<double>(1),
                            tvec.at<double>(2));
    // T_cw: World to Camera
    Sophus::SE3d T_cw(R_eigen, t_eigen);

    // T_wc: Camera to World
    currentFrame_->T_wc = T_cw.inverse();

    // 4. Mark Inliers
    for (int idx : inliers) {
      validFeatures[idx]->isInlier = true;
    }

    return inliers.size();
  }
  return 0;
}
void Frontend::updateObservation() {
  // Update MapPoint observations with current frame features
  for (auto &feat : currentFrame_->featurePtrs) {
    if (!feat->mapPointPtr.expired()) {
      auto mapPoint = feat->mapPointPtr.lock();
      mapPoint->addObserve(feat);
    }
  }
}

void Frontend::createMapPoint() {
  for (size_t i = 0; i < currentFrame_->featurePtrs.size(); ++i) {
    // If we have a match in the right image
    if (currentFrame_->rightFeaturePtrs[i] != nullptr) {
      auto &feature = currentFrame_->featurePtrs[i];
      auto &rightFeature = currentFrame_->rightFeaturePtrs[i];

      // Triangulate
      // Note: pixel2World takes (u,v) and returns (x,y,z) in Camera Frame
      Eigen::Vector3d pointCam =
          camera_->pixel2World(feature->point.pt, rightFeature->point.pt);

      // Transform to World Frame
      // P_w = T_wc * P_c
      Eigen::Vector3d pointWorld = currentFrame_->T_wc * pointCam;

      // Filter invalid depths
      if (pointCam(2) > 0 && pointCam(2) < 100) {
        // Create MapPoint
        auto mapPoint = std::make_shared<MapPoint>(pointWorld);

        // Link Feature -> MapPoint
        feature->mapPointPtr = mapPoint;

        // Add to Map
        map_->addMapPoint(mapPoint);
      }
    }
  }
}

int16_t Frontend::createLeftFeature() {
  // Create a mask to avoid detecting features where we already have them
  cv::Mat mask(currentFrame_->imageL.size(), CV_8UC1, 255);
  for (auto &feat : currentFrame_->featurePtrs) {
    cv::rectangle(mask, feat->point.pt - cv::Point2f(10, 10),
                  feat->point.pt + cv::Point2f(10, 10), 0, cv::FILLED);
  }
  std::vector<cv::KeyPoint> keypoints;
  gftt_->detect(currentFrame_->imageL, keypoints, mask);
  for (auto &kp : keypoints) {
    currentFrame_->featurePtrs.push_back(
        std::make_shared<Feature>(currentFrame_, kp));
  }
  return currentFrame_->featurePtrs.size();
}

int16_t Frontend::matchInRight() {
  std::vector<cv::Point2f> leftKeypoints, rightKeypoints;

  // Prepare inputs for LK Flow
  for (auto &feature : currentFrame_->featurePtrs) {
    leftKeypoints.push_back(feature->point.pt);
    // Initial guess: same position (assuming small motion/rotation)
    rightKeypoints.push_back(feature->point.pt);
  }

  std::vector<uchar> status;
  cv::Mat error;

  // Run LK Optical Flow
  cv::calcOpticalFlowPyrLK(
      currentFrame_->imageL, currentFrame_->imageR, leftKeypoints,
      rightKeypoints, status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      // Create Feature for Right Image
      Feature::Ptr feat(
          new Feature(currentFrame_, cv::KeyPoint(rightKeypoints[i].x,
                                                  rightKeypoints[i].y, 1.0)));
      feat->isLeftFeature = false;
      currentFrame_->rightFeaturePtrs.push_back(std::move(feat));
      num_good_pts++;

      // Verification: Calculate Depth
      // auto depth = camera_->pixel2World(leftKeypoints[i], rightKeypoints[i]);
      // std::cout << "Depth: " << depth(2) << std::endl;

    } else {
      currentFrame_->rightFeaturePtrs.push_back(nullptr);
    }
  }

  std::cout << "Found " << num_good_pts << " matches in the right image."
            << std::endl;

  return num_good_pts;
}
} // namespace StereoSLAM