from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share = get_package_share_directory('vins_fusion_ros2')
    config_file = os.path.join(
        package_share,
        'config',
        'stereo_25mav',
        'stereo_imu_config.yaml'
    )
    rviz_config = os.path.join(
        package_share,
        'config',
        'vins_rviz_config.rviz'
    )

    return LaunchDescription([
        Node(
            package='vins_fusion_ros2',
            executable='stereo_image_splitter',
            name='stereo_image_splitter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_topic': '/usb_cam/image_raw/compressed',
                'left_topic': '/left/image_raw',
                'right_topic': '/right/image_raw',
                'left_frame_id': 'left_camera',
                'right_frame_id': 'right_camera',
                'config_file': config_file,
            }],
        ),
        Node(
            package='vins_fusion_ros2',
            executable='vins_node',
            name='vins_estimator',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True},
                        {'config_file': config_file}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='vins_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        ),
    ])
