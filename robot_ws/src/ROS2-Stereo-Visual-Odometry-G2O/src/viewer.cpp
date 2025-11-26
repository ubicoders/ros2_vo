#include "stereo_visual_slam/viewer.hpp"

namespace StereoSLAM {
Viewer::Viewer(
    std::shared_ptr<Map> map, std::shared_ptr<PinholeCamera> stereoCam,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub,
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub)
    : map_(map), stereoCam_(stereoCam), clock_(clock),
      debugImagePub_(debugImagePub), pointCloudPub_(pointCloudPub),
      pathPub_(pathPub) {
  std::cout << "Viewer Constructor" << std::endl;
}

void Viewer::update() {
  mapPointUpdate();
  pathUpdate();
}

void Viewer::debugImageUpdate(const std::shared_ptr<Frame> frame) {
  cv::Mat debugImage;
  // auto leftImageCol = frame->imageL.cols;
  cv::cvtColor(frame->imageL, debugImage, cv::COLOR_GRAY2BGR);

  for (size_t i = 0; i < frame->leftFeaturePtrs.size(); ++i) {
    auto &keyPoint = frame->leftFeaturePtrs[i];
    if (!keyPoint->isInlier) {
      // color red for non-inliers (in tmers of ransac)
      cv::circle(debugImage, keyPoint->point.pt, 4, cv::Scalar(0, 0, 255), 1,
                 cv::LINE_4, 0);
    } else {
      // color green for inliers (in tmers of ransac)
      cv::circle(debugImage, keyPoint->point.pt, 4, cv::Scalar(0, 255, 0), 1,
                 cv::LINE_4, 0);
    }

    if (keyPoint->mapPointPtr.lock() != nullptr &&
        keyPoint->framePtr.lock() != nullptr) {
      auto mapPoint = keyPoint->mapPointPtr.lock();
      auto frame = keyPoint->framePtr.lock();
      auto uv =
          stereoCam_->world2pixel(mapPoint->getWorldPoint(), frame->T_c2w);
      auto uvPoint = cv::Point2f(uv(0), uv(1));

      // color magenta for map points that are not inliers (in tmers of ransac)
      cv::circle(debugImage, uvPoint, 4, cv::Scalar(255, 0, 255), 1, cv::LINE_4,
                 0);

      // color yellow for map points that are inliers (in tmers of ransac)
      cv::line(debugImage, uvPoint, keyPoint->point.pt, cv::Scalar(0, 255, 255),
               1);
    }
  }

  auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debugImage)
                     .toImageMsg();
  message->header.stamp = clock_->now();
  debugImagePub_->publish(*message);
}

void Viewer::mapPointUpdate() {
  const auto &mapPoints = map_->getMapPoints();

  if (map_->getRequiredViewerUpdated()) {
    auto pointCloud = sensor_msgs::msg::PointCloud2();
    pointCloud.height = 1;
    pointCloud.width = mapPoints.size();
    pointCloud.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(pointCloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(pointCloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pointCloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pointCloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(pointCloud, "rgb");

    // add all map points to point cloud every time it publishes

    // White for stable global map points
    const uint32_t COLOR_WHITE = 0x00FFFFFF; // RGB: 255, 255, 255
    // Red for active local map points (currently being optimized)
    const uint32_t COLOR_RED = 0x00FF0000; // RGB: 255, 0, 0

    for (const auto &[id, mapPoint] : mapPoints) {
      auto vec = mapPoint->getWorldPoint();

      // Set point coordinates
      *iter_x = static_cast<float>(vec(0));
      *iter_y = static_cast<float>(vec(1));
      *iter_z = static_cast<float>(vec(2));

      // Determine the color based on point status
      uint32_t rgb = COLOR_WHITE;
      if (mapPoint->isLocalPoint) {
        rgb = COLOR_RED;
      }

      // Copy color data to the point cloud message
      std::memcpy(&(*iter_rgb), &rgb, sizeof(uint32_t));

      // Advance iterators to the next point
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_rgb;
    }

    pointCloud.header.frame_id = ros2NodeConfig.output_frame_name;
    pointCloud.header.stamp = clock_->now();
    pointCloudPub_->publish(pointCloud);

    pointCloudMsg_ = pointCloud;
    map_->setRequiredViewerUpdated(false);
  } else {
    pointCloudPub_->publish(pointCloudMsg_);
  }
}

void Viewer::pathUpdate() {
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  auto frames = map_->getKeyFrames();
  for (auto &[id, frame] : frames) {
    auto updatePose = frame->T_c2w.inverse();
    auto worldPose = updatePose;

    geometry_msgs::msg::PoseStamped poseStampMsg;
    poseStampMsg.header.stamp = clock_->now();
    poseStampMsg.header.frame_id = ros2NodeConfig.output_frame_name;

    geometry_msgs::msg::Pose pose;
    Eigen::Quaterniond q(worldPose.rotationMatrix());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    pose.position.x = worldPose.translation().x();
    pose.position.y = worldPose.translation().y();
    pose.position.z = worldPose.translation().z();
    poseStampMsg.pose = pose;

    poses.push_back(std::move(poseStampMsg));
  }

  // Print the last pose position only when a new keyframe is added
  if (!frames.empty()) {
    uint32_t lastFrameId = frames.rbegin()->first;
    if (lastFrameId != lastPrintedFrameId_) {
      const auto &lastPose = poses.back().pose;
      std::cout << "Last world pose x: " << lastPose.position.x
                << " y: " << lastPose.position.y
                << " z: " << lastPose.position.z << std::endl;
      lastPrintedFrameId_ = lastFrameId;
    }
  }

  nav_msgs::msg::Path pathMsg;
  pathMsg.header.stamp = clock_->now();
  pathMsg.header.frame_id = ros2NodeConfig.output_frame_name;
  pathMsg.poses = poses;

  pathPub_->publish(pathMsg);
}
} // namespace StereoSLAM