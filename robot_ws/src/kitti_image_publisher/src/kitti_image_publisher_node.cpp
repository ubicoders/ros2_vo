#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cstdlib>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;

class KittiImagePublisher : public rclcpp::Node {
public:
  KittiImagePublisher(const std::string &sequence_arg = "")
      : rclcpp::Node("kitti_stereo_publisher") {
    dataset_root_ = this->declare_parameter<std::string>(
        "dataset_root", "/home/ubuntu/KITTI/odom/dataset");

    std::string default_sequence = sequence_arg;
    if (default_sequence.empty()) {
      default_sequence = "00";
    } else if (default_sequence.length() == 1) {
      default_sequence = "0" + default_sequence;
    }
    sequence_ =
        this->declare_parameter<std::string>("sequence", default_sequence);
    left_folder_ =
        this->declare_parameter<std::string>("left_folder", "image_0");
    right_folder_ =
        this->declare_parameter<std::string>("right_folder", "image_1");
    frame_id_ =
        this->declare_parameter<std::string>("frame_id", "kitti_camera");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 10.0);
    start_index_param_ = this->declare_parameter<int>("start_index", 0);
    repeat_sequence_ = this->declare_parameter<bool>("repeat", false);

    left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/stereo/image_left", 10);
    right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/stereo/image_right", 10);

    loadImagePaths();
    if (left_image_paths_.empty()) {
      throw std::runtime_error(
          "No KITTI images were found with the provided parameters.");
    }

    current_index_ = clampStartIndex(start_index_param_);
    auto interval =
        std::chrono::duration<double>(1.0 / std::max(publish_rate_, 0.1));
    timer_ = this->create_wall_timer(
        interval, std::bind(&KittiImagePublisher::publishNextPair, this));
    RCLCPP_INFO(get_logger(),
                "Streaming KITTI sequence %s (%zu frames) from %s",
                sequence_.c_str(), left_image_paths_.size(),
                (fs::path(dataset_root_) / "sequences" / sequence_).c_str());
  }

private:
  void loadImagePaths() {
    auto seq_path = fs::path(dataset_root_) / "sequences" / sequence_;
    auto left_dir = seq_path / left_folder_;
    auto right_dir = seq_path / right_folder_;

    if (!fs::exists(left_dir) || !fs::is_directory(left_dir)) {
      throw std::runtime_error("Left image directory does not exist: " +
                               left_dir.string());
    }
    if (!fs::exists(right_dir) || !fs::is_directory(right_dir)) {
      throw std::runtime_error("Right image directory does not exist: " +
                               right_dir.string());
    }

    std::vector<fs::path> filenames;
    for (const auto &entry : fs::directory_iterator(left_dir)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      filenames.emplace_back(entry.path().filename());
    }
    std::sort(filenames.begin(), filenames.end());

    for (const auto &filename : filenames) {
      auto left_path = left_dir / filename;
      auto right_path = right_dir / filename;
      if (!fs::exists(right_path)) {
        RCLCPP_WARN(get_logger(),
                    "Missing matching right image for %s, skipping.",
                    filename.c_str());
        continue;
      }
      left_image_paths_.push_back(left_path);
      right_image_paths_.push_back(right_path);
    }
  }

  size_t clampStartIndex(int start_index) const {
    if (left_image_paths_.empty()) {
      return 0;
    }
    if (start_index < 0) {
      return 0;
    }
    auto max_index = static_cast<int>(left_image_paths_.size() - 1);
    return static_cast<size_t>(std::min(start_index, max_index));
  }

  sensor_msgs::msg::Image::SharedPtr
  loadImageMessage(const fs::path &image_path) {
    cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_UNCHANGED);
    if (image.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to read image %s", image_path.c_str());
      return nullptr;
    }

    cv::Mat bgr_image;
    if (image.channels() == 1) {
      cv::cvtColor(image, bgr_image, cv::COLOR_GRAY2BGR);
    } else if (image.channels() == 3) {
      bgr_image = image;
    } else if (image.channels() == 4) {
      cv::cvtColor(image, bgr_image, cv::COLOR_BGRA2BGR);
    } else {
      RCLCPP_ERROR(get_logger(), "Unsupported channel count (%d) for %s",
                   image.channels(), image_path.c_str());
      return nullptr;
    }

    std_msgs::msg::Header header;
    header.frame_id = frame_id_;
    header.stamp = this->get_clock()->now();

    cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8,
                                bgr_image);
    return cv_image.toImageMsg();
  }

  void publishNextPair() {
    if (current_index_ >= left_image_paths_.size()) {
      if (repeat_sequence_) {
        current_index_ = 0;
      } else {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *this->get_clock(), 2000,
            "Reached the end of the sequence, stopping publisher.");
        return;
      }
    }

    auto left_msg = loadImageMessage(left_image_paths_[current_index_]);
    auto right_msg = loadImageMessage(right_image_paths_[current_index_]);
    if (left_msg && right_msg) {
      left_pub_->publish(*left_msg);
      right_pub_->publish(*right_msg);
      ++current_index_;
    } else {
      RCLCPP_WARN(get_logger(), "Dropping frame %zu due to load failure.",
                  current_index_);
      ++current_index_;
    }
  }

  std::string dataset_root_;
  std::string sequence_;
  std::string left_folder_;
  std::string right_folder_;
  std::string frame_id_;
  double publish_rate_;
  int start_index_param_;
  bool repeat_sequence_;

  size_t current_index_{0};
  std::vector<fs::path> left_image_paths_;
  std::vector<fs::path> right_image_paths_;

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

  try {
    auto node = std::make_shared<KittiImagePublisher>(sequence_arg);
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("kitti_stereo_publisher"),
                 "Failed to start KITTI publisher: %s", ex.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
