#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class StereoImageSplitter : public rclcpp::Node {
 public:
  StereoImageSplitter() : Node("stereo_image_splitter") {
    input_topic_ = declare_parameter<std::string>(
        "input_topic", "/usb_cam/image_raw/compressed");
    left_topic_ =
        declare_parameter<std::string>("left_topic", "/left/image_raw");
    right_topic_ =
        declare_parameter<std::string>("right_topic", "/right/image_raw");
    left_frame_id_ =
        declare_parameter<std::string>("left_frame_id", "left_camera");
    right_frame_id_ =
        declare_parameter<std::string>("right_frame_id", "right_camera");
    config_file_ = declare_parameter<std::string>("config_file", "");
    scale_ = declare_parameter<double>("scale", 0.5);
    freq_ = declare_parameter<double>("freq", -1.0);
    if (freq_ < 0.0) {
      freq_ = readFreqFromConfig(config_file_);
    }
    if (scale_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "scale must be greater than 0");
      throw std::runtime_error("invalid scale");
    }
    if (freq_ < 0.0) {
      RCLCPP_FATAL(get_logger(), "freq must be non-negative");
      throw std::runtime_error("invalid freq");
    }
    const auto qos = rclcpp::SensorDataQoS();
    left_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(left_topic_, qos);
    right_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(right_topic_, qos);
    subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        input_topic_, qos,
        std::bind(&StereoImageSplitter::imageCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Splitting %s -> %s and %s, scale=%.3f, freq=%.3f",
                input_topic_.c_str(), left_topic_.c_str(), right_topic_.c_str(),
                scale_, freq_);
  }

 private:
  double readFreqFromConfig(const std::string &config_file) const {
    if (config_file.empty()) {
      return 0.0;
    }

    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_WARN(get_logger(), "Failed to open config file %s; publishing all frames",
                  config_file.c_str());
      return 0.0;
    }

    const cv::FileNode freq_node = fs["freq"];
    if (freq_node.empty()) {
      return 0.0;
    }
    return static_cast<double>(freq_node);
  }

  bool shouldPublish(const std_msgs::msg::Header &header) {
    if (freq_ <= 0.0) {
      return true;
    }

    constexpr double kTimestampEpsilon = 1e-9;
    const double period = 1.0 / freq_;
    const double timestamp =
        static_cast<double>(header.stamp.sec) + header.stamp.nanosec * 1e-9;
    if (std::isnan(next_publish_time_) || timestamp < next_publish_time_ - period) {
      next_publish_time_ = timestamp + period;
      return true;
    }

    if (timestamp + kTimestampEpsilon < next_publish_time_) {
      return false;
    }

    do {
      next_publish_time_ += period;
    } while (next_publish_time_ <= timestamp + kTimestampEpsilon);
    return true;
  }

  cv::Mat resizeImage(const cv::Mat &image) const {
    const int width =
        std::max(1, static_cast<int>(std::round(image.cols * scale_)));
    const int height =
        std::max(1, static_cast<int>(std::round(image.rows * scale_)));
    if (width == image.cols && height == image.rows) {
      return image;
    }

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(width, height), 0.0, 0.0,
               cv::INTER_AREA);
    return resized;
  }

  static sensor_msgs::msg::Image toImageMessage(
      const cv::Mat &image, const std_msgs::msg::Header &source_header,
      const std::string &frame_id) {
    sensor_msgs::msg::Image message;
    message.header = source_header;
    message.header.frame_id = frame_id;
    message.height = static_cast<uint32_t>(image.rows);
    message.width = static_cast<uint32_t>(image.cols);
    message.encoding = sensor_msgs::image_encodings::BGR8;
    message.is_bigendian = false;
    message.step = static_cast<uint32_t>(image.cols * image.elemSize());
    message.data.resize(static_cast<size_t>(message.step) * message.height);
    for (int row = 0; row < image.rows; ++row) {
      std::memcpy(message.data.data() + static_cast<size_t>(row) * message.step,
                  image.ptr(row), message.step);
    }
    return message;
  }

  void imageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
    if (!shouldPublish(message->header)) {
      return;
    }

    const cv::Mat encoded(1, static_cast<int>(message->data.size()), CV_8UC1,
                          const_cast<unsigned char *>(message->data.data()));
    const cv::Mat stereo = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (stereo.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Failed to decode compressed stereo image");
      return;
    }
    if (stereo.cols % 2 != 0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Stereo image width %d is not even; dropping frame", stereo.cols);
      return;
    }

    const int eye_width = stereo.cols / 2;
    const cv::Mat left = stereo(cv::Rect(0, 0, eye_width, stereo.rows));
    const cv::Mat right =
        stereo(cv::Rect(eye_width, 0, eye_width, stereo.rows));
    const cv::Mat left_resized = resizeImage(left);
    const cv::Mat right_resized = resizeImage(right);
    left_publisher_->publish(
        toImageMessage(left_resized, message->header, left_frame_id_));
    right_publisher_->publish(
        toImageMessage(right_resized, message->header, right_frame_id_));

    if (!reported_dimensions_) {
      RCLCPP_INFO(get_logger(),
                  "Decoded combined image %dx%d; publishing each eye as %dx%d",
                  stereo.cols, stereo.rows, left_resized.cols,
                  left_resized.rows);
      reported_dimensions_ = true;
    }
  }

  std::string input_topic_;
  std::string left_topic_;
  std::string right_topic_;
  std::string left_frame_id_;
  std::string right_frame_id_;
  std::string config_file_;
  double scale_;
  double freq_;
  double next_publish_time_{std::numeric_limits<double>::quiet_NaN()};
  bool reported_dimensions_{false};
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoImageSplitter>());
  rclcpp::shutdown();
  return 0;
}
