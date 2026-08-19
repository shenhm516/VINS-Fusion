# VINS-Fusion ROS2 VIO

This repository contains the visual-inertial odometry portion of VINS-Fusion,
adapted for ROS 2 Humble. Loop closure and GPS/global fusion are not included.

Supported sensor configurations:

- monocular camera + IMU
- stereo cameras + IMU
- stereo cameras without IMU

## Dependencies

- ROS 2 Humble
- Ceres Solver 2.x
- Eigen3, Boost, glog
- OpenCV 4.8

The node converts `sensor_msgs/msg/Image` directly and intentionally does not
link `cv_bridge`, because this system's ROS binary is linked to OpenCV 4.5 while
the development environment uses OpenCV 4.8.

## Build

```bash
cd /home/cartin/work/vins_ws
colcon build --packages-select vins_fusion_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

The supplied launch file uses the EuRoC monocular+IMU configuration:

```bash
ros2 launch vins_fusion_ros2 vins_fusion_ros2.launch.py
```

To use another configuration:

```bash
ros2 run vins_fusion_ros2 vins_node --ros-args \
  -p config_file:=/absolute/path/to/config.yaml \
  -p world_frame_id:=world \
  -p body_frame_id:=body \
  -p camera_frame_id:=camera
```

Play a ROS 2 bag in another terminal and set `use_sim_time` when the bag
publishes `/clock`:

```bash
ros2 bag play /path/to/bag --clock
```

The principal outputs are `odometry`, `path`, `imu_propagate`, `point_cloud`,
`margin_cloud`, and `image_track`, under the node namespace if one is supplied.

## Configuration

Example sensor and camera calibration files are under `config/`. Camera and IMU
timestamps must use the same clock. Hardware synchronization and global-shutter
cameras are strongly recommended.

## License

GPLv3. The estimator is derived from the original HKUST VINS-Fusion project.
