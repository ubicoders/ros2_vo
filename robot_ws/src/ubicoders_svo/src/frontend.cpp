#include "svo/frontend.hpp"
#include <cstdint>

namespace UbiSVO {

Frontend::Frontend(std::shared_ptr<StereoGeometry> stereoCam,
                   std::shared_ptr<Map> map, std::shared_ptr<Backend> backend)
    : stereoCam(stereoCam), map(map), backend(backend) {
  std::cout << "FrontEnd Constructor" << std::endl;
  gftt = cv::GFTTDetector::create(150, 0.01, 20);
  orb = cv::ORB::create(2000);
}

bool Frontend::step(std::shared_ptr<Frame> frame) {
  // std::cout << "Input Frame" << std::endl;
  currentFrame = frame;

  if (status_ == Status::INIT) {
    init();
  } else {
    tracking();
  }

  prevFrame = currentFrame;
  return true;
}

void Frontend::tracking() {
  int16_t trackedFeatureCount = trackingFeature();
  int16_t inlierFeatureCount = estimatePose();
  std::cout << "Tracking: " << trackedFeatureCount
            << "/ Inliers: " << inlierFeatureCount << std::endl;

  if (inlierFeatureCount <= 70) {
    std::cout << "Set Keyframe #" << currentFrame->getFrameId() << std::endl;
    currentFrame->setKeyFrame();

    // trackingFeature() has already added the each feature's corresponding map
    // point.
    // updateObservation() will add the each feature's corresponding map point's
    // feature pointer to it.
    // this is becuase all frames' feature has mappoints. but the map point's
    // feature pointer is empty until it is the active keyframe.
    updateObservation();
    // also, map has total active keyframes list.
    map->addKeyframe(currentFrame);

    if (use_backend) {
      backend->updateMap();
    }

    // since this is the new keyframe, create a new set of features for the
    // nextframe to track.
    createLeftFeature();
    // stereo matching to find 3d points as this is a keyframe.
    matchInRight();
    create3DMapPoint();
  }
}

void Frontend::init() {
  currentFrame->T_w2c = Sophus::SE3d();
  // currentFrame->T_d = Sophus::SE3d();
  createLeftFeature();
  matchInRight();
  create3DMapPoint();

  currentFrame->setKeyFrame();
  map->addKeyframe(currentFrame);

  std::cout << "Frontend: Init" << std::endl;
  status_ = Status::TRACKING;
  isFilterInitialized = false;
}

int16_t Frontend::createLeftFeature() {
  // create mask first to avoid tracking the same feature.
  cv::Mat mask(currentFrame->imageL.size(), CV_8UC1, 255);
  for (auto &feat : currentFrame->leftFeaturePtrs) {
    cv::rectangle(mask, feat->point.pt - cv::Point2f(10, 10),
                  feat->point.pt + cv::Point2f(10, 10), 0, cv::FILLED);
  }

  // detect features using mask. The mask will avoid detecting the same feature.
  std::vector<cv::KeyPoint> keypoints;
  // detect features using GFTT detector.
  gftt->detect(currentFrame->imageL, keypoints, mask);
  // orb->detect(currentFrame->imageL, keypoints, mask);

  // adding the detected features to the current frame.
  for (auto &kp : keypoints) {
    currentFrame->leftFeaturePtrs.push_back(
        std::make_shared<Feature>(currentFrame, kp));
  }

  return currentFrame->leftFeaturePtrs.size();
}

int16_t Frontend::matchInRight() {
  // stereo matching to find 3d points as this is a keyframe.
  std::vector<cv::Point2f> leftKeypoints, rightKeypoints;

  for (auto &feature : currentFrame->leftFeaturePtrs) {
    // left keypoints.
    leftKeypoints.push_back(feature->point.pt);
    // initial guess for optical flow to find the right keypoints.
    rightKeypoints.push_back(feature->point.pt);
  }

  std::vector<uchar> status;
  cv::Mat error;
  // double epsilon = 1.0;
  cv::calcOpticalFlowPyrLK(
      currentFrame->imageL, currentFrame->imageR, leftKeypoints, rightKeypoints,
      status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      Feature::Ptr feat(
          new Feature(currentFrame, cv::KeyPoint(rightKeypoints[i].x,
                                                 rightKeypoints[i].y, 1.0)));
      feat->isLeftFeature = false;
      currentFrame->rightFeaturePtrs.push_back(std::move(feat));
      num_good_pts++;
    } else {
      currentFrame->rightFeaturePtrs.push_back(nullptr);
    }
  }
  std::cout << "Find " << num_good_pts << " in the right image." << std::endl;
  return num_good_pts;
}

int16_t Frontend::trackingFeature() {
  if (prevFrame == nullptr) {
    status_ = Status::INIT;
    return -1;
  }
  std::vector<cv::Point2f> prevKeypoints, currKeypoints;

  for (auto &feature : prevFrame->leftFeaturePtrs) {
    prevKeypoints.push_back(feature->point.pt);
    currKeypoints.push_back(feature->point.pt);
  }
  std::vector<uchar> status;
  cv::Mat error;

  cv::calcOpticalFlowPyrLK(
      prevFrame->imageL, currentFrame->imageL, prevKeypoints, currKeypoints,
      status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  // for all previous features, if they are in the current frame, add them to
  // the current frame
  for (size_t i = 0; i < status.size(); ++i) {
    const auto &prevKeypoint = prevFrame->leftFeaturePtrs[i];
    if (status[i] && !prevKeypoint->mapPointPtr.expired()) {
      // if the feature is in the current frame, add it to the current frame.
      // create a New Feature.
      Feature::Ptr feat(
          new Feature(currentFrame, cv::KeyPoint(currKeypoints[i].x,
                                                 currKeypoints[i].y, 1.0)));
      // set the 3d point (map point) to the new feature.
      // The 3d points should be the same as the previous frame.
      // (but after the pose estim, some 3d points may be removed.)
      feat->mapPointPtr = prevKeypoint->mapPointPtr;
      currentFrame->leftFeaturePtrs.push_back(std::move(feat));
    }
  }

  return currentFrame->leftFeaturePtrs.size();
}

void Frontend::create3DMapPoint() {
  Sophus::SE3d T_w2c = currentFrame->T_w2c.inverse();

  std::vector<cv::Point2f> leftPoints, rightPoints;
  std::vector<Feature::Ptr> features;

  //  for all features in the current left frame,
  for (size_t i = 0; i < currentFrame->leftFeaturePtrs.size(); ++i) {

    // if the feature is in the right frame and the feature is not a map point,
    // add it to the leftPoints and rightPoints.
    if (currentFrame->rightFeaturePtrs[i] != nullptr &&
        currentFrame->leftFeaturePtrs[i]->mapPointPtr.expired()) {
      auto &feature = currentFrame->leftFeaturePtrs[i];
      auto &rightFeature = currentFrame->rightFeaturePtrs[i];

      leftPoints.push_back(stereoCam->pixel2camera(feature->point.pt));
      rightPoints.push_back(stereoCam->pixel2camera(rightFeature->point.pt));
      features.push_back(feature);
    }
  }

  cv::Mat T1 = stereoCam->get_T1();
  cv::Mat T2 = stereoCam->get_T2();

  cv::Mat worldHomoPoints;

  cv::triangulatePoints(T1, T2, leftPoints, rightPoints, worldHomoPoints);

  for (int i = 0; i < worldHomoPoints.cols; ++i) {
    cv::Mat x = worldHomoPoints.col(i);
    Eigen::Vector4d homogenousWorldPoint = Eigen::Vector4d::Identity();
    Eigen::Vector3d worldPoint = Eigen::Vector3d::Zero();

    x /= x.at<float>(3, 0);

    homogenousWorldPoint(0) = x.at<float>(0, 0);
    homogenousWorldPoint(1) = x.at<float>(1, 0);
    homogenousWorldPoint(2) = x.at<float>(2, 0);
    homogenousWorldPoint(3) = 1.0;

    if (homogenousWorldPoint(2) > 0 && homogenousWorldPoint(2) <= 50) {
      homogenousWorldPoint = T_w2c * homogenousWorldPoint;

      worldPoint(0) = homogenousWorldPoint(0);
      worldPoint(1) = homogenousWorldPoint(1);
      worldPoint(2) = homogenousWorldPoint(2);

      auto mapPointPtr = std::make_shared<MapPoint>(worldPoint);
      mapPointPtr->isLocalPoint = true;
      mapPointPtr->addObserve(features[i]);

      features[i]->mapPointPtr = mapPointPtr;
      map->addMapPoint(mapPointPtr);
    }
  }
}

int16_t Frontend::estimatePose() {
  cv::Mat rVec;
  cv::Mat tVec;

  std::vector<int>
      inliers; // indices of features where the feature is inlier ()
  std::vector<Feature::Ptr> features;
  std::vector<cv::Point3f> worldPoints;
  std::vector<cv::Point2f> pixelPoints;

  for (auto &feature : currentFrame->leftFeaturePtrs) {
    // it checks if this feature is already associated with a map point.
    if (feature->mapPointPtr.lock() != nullptr) {
      auto mapPoint = feature->mapPointPtr.lock();
      // get the feature's 3d point in the world coordinate.
      auto &worldPoint = mapPoint->getWorldPoint();
      // put this feature for pnpransac
      features.push_back(feature);
      // put the pixel point to match the world point
      pixelPoints.push_back(feature->point.pt);
      // put the world point to match the pixel point
      worldPoints.push_back(
          cv::Point3f(worldPoint(0), worldPoint(1), worldPoint(2)));
      // inliner for the ransac
      feature->isInlier = false;
    }
  }

  auto K = stereoCam->get_K();
  auto distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // distortion coefficients
  // std::cout << "worldPoints size: " << worldPoints.size() << std::endl;
  // std::cout << "pixelPoints size: " << pixelPoints.size() << std::endl;

  if (worldPoints.size() < 4) {
    std::cout << "Not enough points for solvePnP: " << worldPoints.size()
              << std::endl;
    return 0;
  }

  bool success = cv::solvePnPRansac(worldPoints, pixelPoints, K, distCoeffs,
                                    rVec, tVec, false, 100, 2.0, 0.99, inliers,
                                    cv::SOLVEPNP_ITERATIVE);

  std::cout << "success: " << success << std::endl;

  if (success) {
    cv::Mat R;
    Eigen::Matrix3d eigenR;
    Eigen::Vector3d eigenT;
    // // Low-pass filter application with Outlier Rejection
    // if (!isFilterInitialized) {
    //   rVec.copyTo(rVecFiltered);
    //   tVec.copyTo(tVecFiltered);
    //   isFilterInitialized = true;
    // } else {
    //   double rNorm = cv::norm(rVec);
    //   double tNorm = cv::norm(tVec);
    //   double rFilteredNorm = cv::norm(rVecFiltered);
    //   double tFilteredNorm = cv::norm(tVecFiltered);

    //   // Check for outliers (1.5x magnitude jump)
    //   if (rNorm > 0.1 * rFilteredNorm && tNorm > 1.5 * tFilteredNorm) {
    //     std::cout << "Outlier detected! Ignoring jump. rNorm: " << rNorm
    //               << " (ref: " << rFilteredNorm << "), tNorm: " << tNorm
    //               << " (ref: " << tFilteredNorm << ")" << std::endl;
    //     // Do NOT update the filter; keep using the old safe value.
    //   } else {
    //     // Safe to update
    //     rVecFiltered = filterAlpha * rVec + (1.0 - filterAlpha) *
    //     rVecFiltered; tVecFiltered = filterAlpha * tVec + (1.0 - filterAlpha)
    //     * tVecFiltered;
    //   }
    // }

    // // create rotation matrix from rotation vector using filtered values
    cv::Rodrigues(rVec, R);

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        eigenR(i, j) = R.at<double>(i, j);
      }
    }

    eigenT(0) = tVec.at<double>(0, 0);
    eigenT(1) = tVec.at<double>(1, 0);
    eigenT(2) = tVec.at<double>(2, 0);

    // set the inliers
    for (auto &inlier : inliers) {
      features[inlier]->isInlier = true;
    }

    // set the non-inliers
    for (auto &feature : features) {
      if (!feature->isInlier) {
        // remove the map point from the feature
        feature->mapPointPtr.reset();
      }
    }

    currentFrame->T_w2c = Sophus::SE3d(eigenR, eigenT);

  } else {
    // Pose estimation failed - use previous frame's pose as fallback
    // to avoid setting position to (0,0,0) which corrupts the trajectory
    std::cout << "pose estimate failed!!" << std::endl;
    if (prevFrame != nullptr) {
      currentFrame->T_w2c = prevFrame->T_w2c;
      std::cout << "Using previous frame's pose as fallback" << std::endl;
    } else {
      // No previous frame - set to identity (should only happen at startup)
      currentFrame->T_w2c = Sophus::SE3d();
      std::cout << "No previous frame - setting to identity" << std::endl;
    }
  }

  return inliers.size();
}

void Frontend::updateObservation() {
  // for all features in the current keyframe, update their observations.
  // The meaning of the observation is the map point being observed by the
  // feature
  for (auto &feature : currentFrame->leftFeaturePtrs) {
    if (!feature->mapPointPtr.expired()) {
      auto mapPointPtr = feature->mapPointPtr.lock();
      mapPointPtr->addObserve(feature);
    }
  }
}

} // namespace UbiSVO