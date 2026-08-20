#include <vins_fusion_ros2/vins_estimator.h>
#include <glog/logging.h>

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  // ROS 2 launch captures the process console, while glog writes INFO messages
  // to log files by default. Mirror all VINS logs to stderr so timing messages
  // are visible both with `ros2 run` and `ros2 launch`.
  FLAGS_alsologtostderr = true;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VinsEstimator>();
  // 创建多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return 0;
}
