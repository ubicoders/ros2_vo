#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

#include <iomanip>
#include <opencv2/calib3d.hpp>

class EurocImagePublisher : public rclcpp::Node {
public:
  EurocImagePublisher(const std::string &sequence_arg = "")
      : rclcpp::Node("euroc_stereo_publisher") {

    // 1. Resolve sequence name
    dataset_root_ = this->declare_parameter<std::string>("dataset_root",
                                                         "/home/ubuntu/euroc");
    sequence_ = sequence_arg;
    if (sequence_.empty()) {
      sequence_ =
          this->declare_parameter<std::string>("sequence", "MH_01_easy");
    }

    // Map short codes
    std::map<std::string, std::string> sequence_map = {
        {"m1", "MH_01_easy"},      {"m2", "MH_02_easy"},
        {"m3", "MH_03_medium"},    {"m4", "MH_04_difficult"},
        {"m5", "MH_05_difficult"}, {"v1", "V1_01_easy"},
        {"v1_02", "V1_02_medium"}, {"v1_03", "V1_03_difficult"},
        {"v2", "V2_01_easy"},      {"v2_02", "V2_02_medium"},
    };

    if (sequence_map.count(sequence_)) {
      sequence_ = sequence_map[sequence_];
    }

    publish_rate_ = this->declare_parameter<double>("publish_rate", 20.0);

    // 2. Load Camera Parameters and Compute Rectification
    if (!loadAndRectify()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize stereo rectification!");
      // We might want to exit or throw
      return;
    }

    // 3. Load Image Paths
    loadImages();

    // 4. Create Publishers
    left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/stereo/image_left", 10);
    right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/stereo/image_right", 10);

    auto interval = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
        interval, std::bind(&EurocImagePublisher::publishNextPair, this));

    RCLCPP_INFO(get_logger(),
                "Streaming Euroc sequence %s (%zu frames) from %s",
                sequence_.c_str(), left_image_paths_.size(),
                (fs::path(dataset_root_) / sequence_).c_str());
  }

private:
  // Helper to read simple Matrix from YAML list
  cv::Mat readTransform(const std::vector<double> &data) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    for (int i = 0; i < 16; ++i) {
      T.at<double>(i / 4, i % 4) = data[i];
    }
    return T;
  }

  bool loadAndRectify() {
    auto cam0_yaml =
        fs::path(dataset_root_) / sequence_ / "mav0" / "cam0" / "sensor.yaml";
    auto cam1_yaml =
        fs::path(dataset_root_) / sequence_ / "mav0" / "cam1" / "sensor.yaml";

    if (!fs::exists(cam0_yaml) || !fs::exists(cam1_yaml)) {
      RCLCPP_ERROR(get_logger(), "Could not find sensor.yaml files for %s",
                   sequence_.c_str());
      return false;
    }

    try {
      YAML::Node config0 = YAML::LoadFile(cam0_yaml.string());
      YAML::Node config1 = YAML::LoadFile(cam1_yaml.string());

      // Read Resolution
      std::vector<int> res0 = config0["resolution"].as<std::vector<int>>();
      cv::Size imageSize(res0[0], res0[1]);

      // Read Intrinsics [fu, fv, cu, cv]
      std::vector<double> int0 =
          config0["intrinsics"].as<std::vector<double>>();
      std::vector<double> int1 =
          config1["intrinsics"].as<std::vector<double>>();

      cv::Mat K0 = cv::Mat::eye(3, 3, CV_64F);
      K0.at<double>(0, 0) = int0[0];
      K0.at<double>(1, 1) = int0[1];
      K0.at<double>(0, 2) = int0[2];
      K0.at<double>(1, 2) = int0[3];

      cv::Mat K1 = cv::Mat::eye(3, 3, CV_64F);
      K1.at<double>(0, 0) = int1[0];
      K1.at<double>(1, 1) = int1[1];
      K1.at<double>(0, 2) = int1[2];
      K1.at<double>(1, 2) = int1[3];

      // Read Distortion
      std::vector<double> D0_data =
          config0["distortion_coefficients"].as<std::vector<double>>();
      std::vector<double> D1_data =
          config1["distortion_coefficients"].as<std::vector<double>>();
      cv::Mat D0(D0_data);
      cv::Mat D1(D1_data);

      // Read Extrinsics T_BS (Sensor to Body)
      // T_BS means point in sensor frame -> point in body frame: P_B = T_BS *
      // P_S
      cv::Mat T_BS0 =
          readTransform(config0["T_BS"]["data"].as<std::vector<double>>());
      cv::Mat T_BS1 =
          readTransform(config1["T_BS"]["data"].as<std::vector<double>>());

      // We need Transform from Cam0 to Cam1 (P_S1 = R * P_S0 + T)
      // P_B = T_BS0 * P_S0  => P_S0 = inv(T_BS0) * P_B
      // P_B = T_BS1 * P_S1
      // P_S1 = inv(T_BS1) * P_B = inv(T_BS1) * T_BS0 * P_S0
      // So T_01 = inv(T_BS1) * T_BS0

      cv::Mat T_01 = T_BS1.inv() * T_BS0;
      cv::Mat R = T_01(cv::Rect(0, 0, 3, 3));
      cv::Mat T = T_01(cv::Rect(3, 0, 1, 3));

      // Stereo Rectification
      cv::Mat R1, R2, P1, P2, Q;
      cv::stereoRectify(K0, D0, K1, D1, imageSize, R, T, R1, R2, P1, P2, Q,
                        cv::CALIB_ZERO_DISPARITY,
                        0); // alpha=0 to remove black borders

      // Compute maps
      cv::initUndistortRectifyMap(K0, D0, R1, P1, imageSize, CV_16SC2, map1_l_,
                                  map2_l_);
      cv::initUndistortRectifyMap(K1, D1, R2, P2, imageSize, CV_16SC2, map1_r_,
                                  map2_r_);

      // Output new parameters
      double fx_new = P1.at<double>(0, 0);
      double fy_new = P1.at<double>(1, 1);
      double cx_new = P1.at<double>(0, 2);
      double cy_new = P1.at<double>(1, 2);
      double baseline_new =
          -P2.at<double>(0, 3) / P2.at<double>(0, 0); // Tx = -P2_03 / fx

      std::cout << "\n============================================\n";
      std::cout << "RECTIFIED CAMERA PARAMETERS (Update config.hpp !!)\n";
      std::cout << "============================================\n";
      std::cout << "fx:   " << std::fixed << std::setprecision(5) << fx_new
                << "\n";
      std::cout << "fy:   " << fy_new << "\n";
      std::cout << "cx:   " << cx_new << "\n";
      std::cout << "cy:   " << cy_new << "\n";
      std::cout << "baseline: " << baseline_new << "\n";
      std::cout << "============================================\n\n";

      return true;

    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse sensor.yaml: %s", e.what());
      return false;
    }
  }

  void loadImages() {
    auto left_dir =
        fs::path(dataset_root_) / sequence_ / "mav0" / "cam0" / "data";
    auto right_dir =
        fs::path(dataset_root_) / sequence_ / "mav0" / "cam1" / "data";

    if (!fs::exists(left_dir) || !fs::exists(right_dir)) {
      RCLCPP_ERROR(get_logger(), "Image directories not found.");
      return;
    }

    std::vector<fs::path> filenames;
    for (const auto &entry : fs::directory_iterator(left_dir)) {
      if (entry.is_regular_file() && entry.path().extension() == ".png") {
        filenames.push_back(entry.path().filename());
      }
    }
    std::sort(filenames.begin(), filenames.end());

    for (const auto &fn : filenames) {
      auto left_p = left_dir / fn;
      auto right_p = right_dir / fn;
      if (fs::exists(right_p)) {
        left_image_paths_.push_back(left_p);
        right_image_paths_.push_back(right_p);
      }
    }
  }

  void publishNextPair() {
    if (current_index_ >= left_image_paths_.size()) {
      return; // End of sequence
    }

    auto left_msg = loadImageMessage(left_image_paths_[current_index_], true);
    auto right_msg =
        loadImageMessage(right_image_paths_[current_index_], false);

    if (left_msg && right_msg) {
      left_msg->header.stamp = this->now();
      right_msg->header.stamp = left_msg->header.stamp;

      left_pub_->publish(*left_msg);
      right_pub_->publish(*right_msg);
    }
    current_index_++;
  }

  sensor_msgs::msg::Image::SharedPtr loadImageMessage(const fs::path &path,
                                                      bool is_left) {
    cv::Mat img = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    if (img.empty())
      return nullptr;

    // Convert to grayscale if needed (Euroc already mono)
    if (img.channels() > 1) {
      cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }

    // Rectify
    cv::Mat img_rect;
    if (is_left) {
      cv::remap(img, img_rect, map1_l_, map2_l_, cv::INTER_LINEAR);
    } else {
      cv::remap(img, img_rect, map1_r_, map2_r_, cv::INTER_LINEAR);
    }

    std_msgs::msg::Header header;
    header.frame_id = "euroc_camera";

    cv_bridge::CvImage cv_img(header, sensor_msgs::image_encodings::MONO8,
                              img_rect);
    return cv_img.toImageMsg();
  }

  std::string dataset_root_;
  std::string sequence_;
  double publish_rate_;

  std::vector<fs::path> left_image_paths_;
  std::vector<fs::path> right_image_paths_;
  size_t current_index_ = 0;

  // Rectification maps
  cv::Mat map1_l_, map2_l_;
  cv::Mat map1_r_, map2_r_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::string sequence_arg = "";
  if (non_ros_args.size() > 1) {
    sequence_arg = non_ros_args[1];
  }

  auto node = std::make_shared<EurocImagePublisher>(sequence_arg);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
