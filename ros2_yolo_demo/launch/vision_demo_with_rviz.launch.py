from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            output='screen',
            parameters=[{'align_depth': True}]
        ),
        Node(
            package='yolov8_depth_demo',
            executable='yolo_tracker',
            name='yolo_tracker',
            output='screen'
        ),
        Node(
            package='yolov8_depth_demo',
            executable='depth_to_3d_rviz',
            name='depth_to_3d_rviz',
            output='screen'
        ),
        Node(
            package='yolov8_depth_demo',
            executable='performance_monitor',
            name='performance_monitor',
            output='screen'
        ),
        # RViz node (will open RViz)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'rviz/detection.rviz']
        )
    ])
