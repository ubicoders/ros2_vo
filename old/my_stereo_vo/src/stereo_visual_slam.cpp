#include "my_stereo_vo/stereo_visual_slam.hpp"
namespace StereoSLAM {
StereoVisualSLAM::StereoVisualSLAM(const rclcpp::NodeOptions &options)
    : Node("stereo_visual_slam", options) {

  // Hardcoded parameters (KITTI 00-02)
  double focal = 718.8560;
  double baseline = 0.537;
  cv::Point2d pp(607.1928, 185.2157);
  Eigen::Matrix3d K;
  Eigen::Vector3d t;
  K << focal, 0.0, pp.x, 0.0, focal, pp.y, 0.0, 0.0, 1.0;
  t << -386.1448, 0.0, 0.0; // Tx * fx approx

  // Calculate Pose (T_c_w)
  // Note: Original code does some inverse logic here, we simplify for now
  t = K.inverse() * t;
  Sophus::SE3d pose = Sophus::SE3d(Eigen::Matrix3d::Identity(), t);
  // Initialize Camera
  stereoCam_ =
      std::make_shared<PinholeCamera>(focal, focal, pp.x, pp.y, baseline, pose);

  // Initialize Map - step 4
  map_ = std::make_shared<Map>();
  stereoCam_ = std::make_shared<PinholeCamera>(focal, focal, pp.x, pp.y,
                                               baseline, Sophus::SE3d());
  backend_ = std::make_shared<StereoSLAM::Backend>();

  // Initialize Frontend - step 1
  frontend_ = std::make_shared<Frontend>();
  frontend_->setMap(map_);
  frontend_->setCamera(stereoCam_);

  bool use_backend = true; // Toggle this to enable/disable backend
  if (use_backend) {
    frontend_->setBackend(backend_);
    backend_->setMap(map_); // - step 2
  }

  // 1. Initialize Publisher (Exact topic from original)
  debugImagePub_ =
      this->create_publisher<sensor_msgs::msg::Image>("/mono/image", 50);
  // 2. Initialize Subscribers (Exact topics from original)
  auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
  leftImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(
      this, "/stereo/image_left", qos);
  rightImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(
      this, "/stereo/image_right", qos);

  pointCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/mono/pointcloud", 10);

  pathPub_ = this->create_publisher<nav_msgs::msg::Path>("/mono/path", 10);

  // 3. Initialize Synchronizer
  syncStereo_ =
      std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(
          ImageSyncPolicy(10), *leftImageSub_, *rightImageSub_);
  syncStereo_->registerCallback(std::bind(&StereoVisualSLAM::ImageCallback,
                                          this, std::placeholders::_1,
                                          std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Stereo Visual SLAM Node Initialized");
}

void StereoVisualSLAM::ImageCallback(const Image::ConstSharedPtr &leftImage,
                                     const Image::ConstSharedPtr &rightImage) {
  // 1. Create a new Frame
  auto frame = std::make_shared<Frame>();
  // 2. Convert ROS images to OpenCV (Grayscale for processing)
  auto CVImageL = cv_bridge::toCvCopy(leftImage, leftImage->encoding)->image;
  auto CVImageR = cv_bridge::toCvCopy(rightImage, rightImage->encoding)->image;

  cv::cvtColor(CVImageL, frame->imageL, cv::COLOR_BGR2GRAY);
  cv::cvtColor(CVImageR, frame->imageR, cv::COLOR_BGR2GRAY);
  // 3. Run Frontend Step (Detect Features)
  frontend_->step(frame);
  // 4. Visualization (Draw detected features)
  cv::Mat debugImage;
  cv::cvtColor(frame->imageL, debugImage, cv::COLOR_GRAY2BGR);

  pointCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/mono/pointcloud", 10);

  //======= step 4 ======= temporary
  // Create PointCloud2 message
  sensor_msgs::msg::PointCloud2 cloudMsg;
  cloudMsg.header = leftImage->header;
  cloudMsg.height = 1;
  cloudMsg.width = map_->getActiveMapPoints().size();
  pointCloudPub_->publish(cloudMsg);
  // ======= step 4 ======= temporary

  // ======== step 5 ======= temporary
  // Update Path
  geometry_msgs::msg::PoseStamped poseStamped;
  poseStamped.header = leftImage->header;

  // Convert Sophus SE3 to ROS Pose
  Eigen::Vector3d t = frame->T_wc.translation();
  Eigen::Quaterniond q = frame->T_wc.unit_quaternion();

  poseStamped.pose.position.x = t.x();
  poseStamped.pose.position.y = t.y();
  poseStamped.pose.position.z = t.z();
  poseStamped.pose.orientation.x = q.x();
  poseStamped.pose.orientation.y = q.y();
  poseStamped.pose.orientation.z = q.z();
  poseStamped.pose.orientation.w = q.w();
  pathMsg_.header = leftImage->header;
  pathMsg_.poses.push_back(poseStamped);
  pathPub_->publish(pathMsg_);
  // ======= step 5 ======= temporary

  // ... (Simplified construction for verification)
  // For now, let's just print the map size to verify
  std::cout << "Map Size: " << map_->getActiveMapPoints().size() << std::endl;

  for (auto &feature : frame->featurePtrs) {
    cv::circle(debugImage, feature->point.pt, 4, cv::Scalar(0, 255, 0), 1);
  }
  // 5. Publish Debug Image
  sensor_msgs::msg::Image::SharedPtr msg =
      cv_bridge::CvImage(leftImage->header, "bgr8", debugImage).toImageMsg();

  debugImagePub_->publish(*msg);
}
} // namespace StereoSLAM