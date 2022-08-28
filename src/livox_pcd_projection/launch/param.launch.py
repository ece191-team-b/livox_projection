from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="livox_pcd_projection",
            executable="projection",
            name="param_launch",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"intrinsic_path": "calibration_data/parameters/intrinsic.txt"},
                {"extrinsic_path": "calibration_data/parameters/extrinsic.txt"},
                {"camera_topic": "/right_camera/image"},
                {'lidar_topic': "/livox/lidar_3WEDH7600108721"},
                {"lidar_threshold": 100000},
                {"refresh_rate": 10},
                {"debug": False},
            ]
        )
    ])