#include "svo/viewer.hpp"

namespace UbiSVO {
Viewer::Viewer(
    std::shared_ptr<Map> map, std::shared_ptr<StereoGeometry> stereoCam,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub,
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub)
    : map_(map), stereoCam_(stereoCam), clock_(clock),
      debugImagePub_(debugImagePub), pointCloudPub_(pointCloudPub),
      pathPub_(pathPub), markerPub_(markerPub) {
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
      // color red for outliers (in tmers of ransac)
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
          stereoCam_->world2pixel(mapPoint->getWorldPoint(), frame->T_w2c);
      auto uvPoint = cv::Point2f(uv(0), uv(1));

      // color magenta reprojected points.
      cv::circle(debugImage, uvPoint, 4, cv::Scalar(255, 0, 255), 1, cv::LINE_4,
                 0);

      // color yellow for map points that are supposed to be reprojected. to
      // either green or red based on ransac.
      cv::line(debugImage, uvPoint, keyPoint->point.pt, cv::Scalar(0, 255, 255),
               1);
    }
  }

  auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debugImage)
                     .toImageMsg();
  message->header.stamp = clock_->now();
  debugImagePub_->publish(*message);
}

void Viewer::publishCameraFrustum(const Sophus::SE3d &T_w2c) {
  auto K = stereoCam_->get_K();
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  // Assume scale/depth of the frustum
  double z_depth = 0.5; // 0.5 meter frustum depth

  // Image size (approximate or hardcoded from config, or back-calculated from
  // K) we just need valid FOV. width approx 752, height 480
  double w = 752.0;
  double h = 480.0;

  // Frustum corners in Camera Frame
  // Center is (0,0,0)
  std::vector<Eigen::Vector3d> corners_c;
  corners_c.push_back(Eigen::Vector3d(0, 0, 0));
  // Top-Left (0,0)
  corners_c.push_back(Eigen::Vector3d((0 - cx) * z_depth / fx,
                                      (0 - cy) * z_depth / fy, z_depth));
  // Top-Right (w,0)
  corners_c.push_back(Eigen::Vector3d((w - cx) * z_depth / fx,
                                      (0 - cy) * z_depth / fy, z_depth));
  // Bottom-Right (w,h)
  corners_c.push_back(Eigen::Vector3d((w - cx) * z_depth / fx,
                                      (h - cy) * z_depth / fy, z_depth));
  // Bottom-Left (0,h)
  corners_c.push_back(Eigen::Vector3d((0 - cx) * z_depth / fx,
                                      (h - cy) * z_depth / fy, z_depth));

  // Transform to World Frame
  // T_wc = inv(T_w2c)
  Sophus::SE3d T_wc = T_w2c.inverse();
  Eigen::Vector3d p_wc = T_wc.translation();
  Eigen::Matrix3d R_wc = T_wc.rotationMatrix();

  // Rotation for Rviz (Camera -> World Viz)
  Eigen::Matrix3d R_viz;
  R_viz << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  std::vector<Eigen::Vector3d> corners_viz;
  for (const auto &pt : corners_c) {
    // Point in World
    Eigen::Vector3d pt_w = R_wc * pt + p_wc;
    // Point in Viz
    Eigen::Vector3d pt_v = R_viz * pt_w;
    corners_viz.push_back(pt_v);
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = ros2NodeConfig.output_frame_name;
  marker.header.stamp = clock_->now();
  marker.ns = "frustum";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02; // Line width
  marker.color.r = 1.0;
  marker.color.a = 1.0;

  // Center to corners
  for (size_t i = 1; i <= 4; ++i) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = corners_viz[0].x();
    p1.y = corners_viz[0].y();
    p1.z = corners_viz[0].z();
    p2.x = corners_viz[i].x();
    p2.y = corners_viz[i].y();
    p2.z = corners_viz[i].z();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }
  // Connect corners (rectangle)
  for (size_t i = 1; i <= 4; ++i) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = corners_viz[i].x();
    p1.y = corners_viz[i].y();
    p1.z = corners_viz[i].z();
    size_t next = (i % 4) + 1;
    p2.x = corners_viz[next].x();
    p2.y = corners_viz[next].y();
    p2.z = corners_viz[next].z();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  markerPub_->publish(marker);
}

void Viewer::mapPointUpdate() {
  const auto &mapPoints = map_->getMapPoints();

  // Rotation from Camera Frame (Z-forward, Y-down) to ROS World Frame
  // (X-forward, Z-up)
  Eigen::Matrix3d R_viz;
  R_viz << 0, 0, 1, -1, 0, 0, 0, -1, 0;

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

    // White for stable global map points
    const uint32_t COLOR_WHITE = 0x00FFFFFF; // RGB: 255, 255, 255
    // Red for active local map points
    const uint32_t COLOR_RED = 0x00FF0000; // RGB: 255, 0, 0

    for (const auto &[id, mapPoint] : mapPoints) {
      Eigen::Vector3d vec = mapPoint->getWorldPoint();

      // Apply rotation
      vec = R_viz * vec;

      // Set point coordinates
      *iter_x = static_cast<float>(vec(0));
      *iter_y = static_cast<float>(vec(1));
      *iter_z = static_cast<float>(vec(2));

      // Determine the color based on point status
      uint32_t rgb = COLOR_WHITE;
      if (mapPoint->isLocalPoint) {
        rgb = COLOR_RED;
      }

      std::memcpy(&(*iter_rgb), &rgb, sizeof(uint32_t));

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

  // Rotation from Camera Frame (Z-forward, Y-down) to ROS World Frame
  // (X-forward, Z-up)
  Eigen::Matrix3d R_viz;
  R_viz << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  auto frames = map_->getKeyFrames();
  for (auto &[id, frame] : frames) {
    auto updatePose = frame->T_w2c.inverse();

    // Apply rotation to both rotation and translation
    // T_viz = R_viz * T_wc
    // Position: p_viz = R_viz * p_wc
    // Rotation: R_pose_viz = R_viz * R_wc

    Eigen::Vector3d p_viz = R_viz * updatePose.translation();
    Eigen::Matrix3d R_pose_viz = R_viz * updatePose.rotationMatrix();

    geometry_msgs::msg::PoseStamped poseStampMsg;
    poseStampMsg.header.stamp = clock_->now();
    poseStampMsg.header.frame_id = ros2NodeConfig.output_frame_name;

    geometry_msgs::msg::Pose pose;
    Eigen::Quaterniond q(R_pose_viz);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    pose.position.x = p_viz.x();
    pose.position.y = p_viz.y();
    pose.position.z = p_viz.z();
    poseStampMsg.pose = pose;

    poses.push_back(std::move(poseStampMsg));
  }

  if (!frames.empty()) {
    // Publish Frustum for the latest frame
    publishCameraFrustum(frames.rbegin()->second->T_w2c);

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
} // namespace UbiSVO