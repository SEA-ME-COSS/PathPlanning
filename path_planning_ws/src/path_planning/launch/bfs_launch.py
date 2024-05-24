import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    package_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Path to executable
    bfs_node = launch_ros.actions.Node(
        package='path_planning',
        executable='bfs',
        name='bfs',
        output='screen',
        parameters=[
            os.path.join(package_dir, '..', 'config', 'bfs_params.yaml') # 수정된 경로
        ]
    )

    return LaunchDescription([
        bfs_node,
    ])



