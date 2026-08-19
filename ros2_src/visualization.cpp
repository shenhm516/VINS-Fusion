#include <vins_fusion_ros2/visualization.h>

#include <cstring>

#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

TimeStampSec fromMsg(const builtin_interfaces::msg::Time &msg) {
  return static_cast<TimeStampSec>(msg.sec + msg.nanosec * 1e-9);
}

Eigen::Vector3d fromMsg(const geometry_msgs::msg::Vector3 &msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}
Eigen::Quaterniond fromMsg(const geometry_msgs::msg::Quaternion &msg) {
  return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}

IMUData fromMsg(const sensor_msgs::msg::Imu &msg) {
  IMUData imu;
  imu.timestamp = fromMsg(msg.header.stamp);
  imu.angular_velocity = fromMsg(msg.angular_velocity);
  imu.linear_acceleration = fromMsg(msg.linear_acceleration);
  imu.orientation = fromMsg(msg.orientation);
  return imu;
}

cv::Mat fromMsg(const sensor_msgs::msg::Image &img_msg) {
  if (img_msg.height == 0 || img_msg.width == 0 || img_msg.data.empty()) {
    return {};
  }

  const auto &encoding = img_msg.encoding;
  int type = -1;
  if (encoding == sensor_msgs::image_encodings::MONO8 || encoding == "8UC1") {
    type = CV_8UC1;
  } else if (encoding == sensor_msgs::image_encodings::BGR8 ||
             encoding == sensor_msgs::image_encodings::RGB8) {
    type = CV_8UC3;
  } else if (encoding == sensor_msgs::image_encodings::BGRA8 ||
             encoding == sensor_msgs::image_encodings::RGBA8) {
    type = CV_8UC4;
  } else {
    throw std::runtime_error("unsupported image encoding: " + encoding);
  }

  cv::Mat source(static_cast<int>(img_msg.height),
                 static_cast<int>(img_msg.width), type,
                 const_cast<unsigned char *>(img_msg.data.data()), img_msg.step);
  cv::Mat mono;
  if (type == CV_8UC1) {
    mono = source.clone();
  } else if (encoding == sensor_msgs::image_encodings::RGB8) {
    cv::cvtColor(source, mono, cv::COLOR_RGB2GRAY);
  } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
    cv::cvtColor(source, mono, cv::COLOR_RGBA2GRAY);
  } else if (encoding == sensor_msgs::image_encodings::BGRA8) {
    cv::cvtColor(source, mono, cv::COLOR_BGRA2GRAY);
  } else {
    cv::cvtColor(source, mono, cv::COLOR_BGR2GRAY);
  }
  return mono;
}

FeatureFrame fromMsg(const sensor_msgs::msg::PointCloud &msg) {
  FeatureFrame featureFrame;
  for (unsigned int i = 0; i < msg.points.size(); i++) {
    if (msg.channels.size() < 6) {
      continue;
    }
    int feature_id = msg.channels[0].values[i];
    int camera_id = msg.channels[1].values[i];
    double x = msg.points[i].x;
    double y = msg.points[i].y;
    double z = msg.points[i].z;
    double p_u = msg.channels[2].values[i];
    double p_v = msg.channels[3].values[i];
    double velocity_x = msg.channels[4].values[i];
    double velocity_y = msg.channels[5].values[i];
    if (msg.channels.size() > 5) {
      double gx = msg.channels[6].values[i];
      double gy = msg.channels[7].values[i];
      double gz = msg.channels[8].values[i];
      // pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
    }
    // assert(z == 1);
    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
  }
  return featureFrame;
}

builtin_interfaces::msg::Time toMsg(TimeStampSec sec) {
  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = static_cast<int32_t>(sec);
  ros_time.nanosec = static_cast<uint32_t>((sec - ros_time.sec) * 1e9);
  return ros_time;
}

nav_msgs::msg::Odometry toMsg(const OdomData &msg) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = toMsg(msg.timestamp);

  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = msg.position.x();
  odometry.pose.pose.position.y = msg.position.y();
  odometry.pose.pose.position.z = msg.position.z();
  odometry.pose.pose.orientation.x = msg.orientation.x();
  odometry.pose.pose.orientation.y = msg.orientation.y();
  odometry.pose.pose.orientation.z = msg.orientation.z();
  odometry.pose.pose.orientation.w = msg.orientation.w();
  odometry.twist.twist.linear.x = msg.velocity.x();
  odometry.twist.twist.linear.y = msg.velocity.y();
  odometry.twist.twist.linear.z = msg.velocity.z();
  return odometry;
}

sensor_msgs::msg::Image toMsg(const cv::Mat &msg) {
  if (msg.empty() || msg.depth() != CV_8U ||
      (msg.channels() != 1 && msg.channels() != 3)) {
    throw std::runtime_error("track image must be mono8 or bgr8");
  }
  sensor_msgs::msg::Image image;
  image.height = static_cast<uint32_t>(msg.rows);
  image.width = static_cast<uint32_t>(msg.cols);
  image.encoding = msg.channels() == 1 ? sensor_msgs::image_encodings::MONO8
                                       : sensor_msgs::image_encodings::BGR8;
  image.is_bigendian = false;
  image.step = static_cast<uint32_t>(msg.cols * msg.elemSize());
  image.data.resize(static_cast<size_t>(image.step) * image.height);
  for (int row = 0; row < msg.rows; ++row) {
    std::memcpy(image.data.data() + static_cast<size_t>(row) * image.step,
                msg.ptr(row), image.step);
  }
  return image;
}

sensor_msgs::msg::PointCloud toMsg(const PointCloudData &msg) {
  sensor_msgs::msg::PointCloud pointcloud;
  pointcloud.header.stamp = toMsg(msg.timestamp);
  pointcloud.header.frame_id = "world";
  pointcloud.points.reserve(msg.points.size());
  for (const auto &pt : msg.points) {
    geometry_msgs::msg::Point32 pt_ros;
    pt_ros.x = static_cast<float>(pt.x());
    pt_ros.y = static_cast<float>(pt.y());
    pt_ros.z = static_cast<float>(pt.z());
    pointcloud.points.emplace_back(pt_ros);
  }

  pointcloud.channels.reserve(msg.channels.size());
  for (const auto &ch : msg.channels) {
    sensor_msgs::msg::ChannelFloat32 ch_ros;
    ch_ros.name = ch.name;
    ch_ros.values = ch.values;
    pointcloud.channels.emplace_back(std::move(ch_ros));
  }

  return pointcloud;
}
