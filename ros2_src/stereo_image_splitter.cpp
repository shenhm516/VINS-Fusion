#include <cstring>
#include <memory>
#include <string>

#include <opencv2/imgcodecs.hpp>
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
    const auto qos = rclcpp::SensorDataQoS();
    left_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(left_topic_, qos);
    right_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(right_topic_, qos);
    subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        input_topic_, qos,
        std::bind(&StereoImageSplitter::imageCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Splitting %s -> %s and %s",
                input_topic_.c_str(), left_topic_.c_str(), right_topic_.c_str());
  }

 private:
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
    left_publisher_->publish(
        toImageMessage(left, message->header, left_frame_id_));
    right_publisher_->publish(
        toImageMessage(right, message->header, right_frame_id_));

    if (!reported_dimensions_) {
      RCLCPP_INFO(get_logger(),
                  "Decoded combined image %dx%d; publishing each eye as %dx%d",
                  stereo.cols, stereo.rows, eye_width, stereo.rows);
      reported_dimensions_ = true;
    }
  }

  std::string input_topic_;
  std::string left_topic_;
  std::string right_topic_;
  std::string left_frame_id_;
  std::string right_frame_id_;
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
