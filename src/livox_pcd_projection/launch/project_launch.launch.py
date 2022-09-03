from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
     
    config = os.path.join(get_package_share_directory('livox_pcd_projection'), 'config', 'config.yaml',
    'config',
    'project.yaml')
    
    
    return LaunchDescription([      
    Node(package='livox_pcd_projection',
            name = "projector",
            executable='projector',
            parameters = [config]), 
    Node(package='livox_pcd_projection',
            name = "stamper",
            executable='stamper',
            parameters = [config]),

]) 