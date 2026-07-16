from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    package_share = Path(get_package_share_directory('underwater_navigation'))
    rviz_config = package_share / 'config' / 'navigation.rviz'

    # 你的 SLAM 节点
    nav_node = Node(
        package='underwater_navigation',
        executable='navigation_node',
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(rviz_config)],
        output='screen'
    )

    # （可选）自动播放bag
    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            '/home/xiaoyang/aqualoc_ros2_bags/harbor_sequence_1'
        ],
        output='screen'
    )

    return LaunchDescription([
        nav_node,
        rviz_node,
        bag_play
    ])
