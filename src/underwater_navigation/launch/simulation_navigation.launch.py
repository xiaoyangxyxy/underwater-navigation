from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = Path(get_package_share_directory('underwater_navigation'))
    simulation_share = Path(get_package_share_directory('underwater_simulation'))

    rviz_config = navigation_share / 'config' / 'navigation.rviz'
    tunnel_launch = simulation_share / 'launch' / 'tunnel_world.launch.py'
    robot_urdf = simulation_share / 'urdf' / 'pipe_robot.urdf'
    robot_description = robot_urdf.read_text()
    record_slam_debug = LaunchConfiguration('record_slam_debug')

    common_params = {
        'use_sim_time': True,
    }

    trajectory_params = {
        'start_x': -14.0,
        'start_y': 0.0,
        'start_z': 0.0,
        'forward_speed': 0.2,
        'lateral_amp': 0.3,
        'lateral_freq': 0.2,
        'depth_amp': 0.2,
        'depth_freq': 0.15,
        'yaw_amp': 0.2,
        'yaw_freq': 0.1,
        'pitch_amp': 0.05,
        'pitch_freq': 0.2,
        'roll_amp': 0.05,
        'roll_freq': 0.25,
    }

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(tunnel_launch))
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            **common_params,
            'robot_description': robot_description,
        }]
    )

    simulation_motion_node = Node(
        package='underwater_navigation',
        executable='simulation_motion_node',
        name='simulation_motion_node',
        output='screen',
        parameters=[{
            **common_params,
            **trajectory_params,
            'robot_model_name': 'pipe_robot',
            'set_entity_state_service': '/set_entity_state',
            'dvl_topic': '/dvl',
            'dvl_noise_std': 0.02,
            'dvl_rate_hz': 10.0,
            'publish_clock': False,
            'use_relative_ground_truth': True,
        }]
    )

    simulation_sensor_node = Node(
        package='underwater_navigation',
        executable='simulation_sensor_node',
        name='simulation_sensor_node',
        output='screen',
        parameters=[{
            **common_params,
            **trajectory_params,
            'imu_topic': '/imu',
            'depth_topic': '/depth',
            'mag_topic': '/mag',
            'publish_clock': False,
            'imu_offset_x': 0.0,
            'imu_offset_y': 0.0,
            'imu_offset_z': 0.08,
            'imu_acc_noise_std': 0.02,
            'imu_gyro_noise_std': 0.001,
            'depth_noise_std': 0.02,
            'mag_noise_std': 0.01,
        }]
    )

    navigation_node = Node(
        package='underwater_navigation',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{
            **common_params,
            'imu_topic': '/imu',
            'depth_topic': '/depth',
            'dvl_topic': '/dvl',
            'mag_topic': '/mag',
            'ground_truth_pose_topic': '/ground_truth/pose',
            'path_topic': '/slam_path',
            'slam_debug_topic': '/slam_debug',
            'imu_window_size': 50,
            'initial_velocity_prior_sigma': 1.0,
            'dvl_match_max_dt': 0.10,
            'dvl_displacement_factor_enabled': True,
            'dvl_displacement_sigma': 0.05,
            'debug_attitude_prior_enabled': False,
            'debug_attitude_prior_sigma': 0.02,
        }]
    )

    slam_debug_recorder_node = Node(
        package='underwater_navigation',
        executable='slam_debug_recorder_node',
        name='slam_debug_recorder_node',
        output='screen',
        condition=IfCondition(record_slam_debug),
        parameters=[{
            'slam_debug_topic': '/slam_debug',
            'output_csv_path': (
                '/home/xiaoyang/water_robot/auv_ws/'
                'slam_logs/slam_debug_6dof.csv'),
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_config)],
        output='screen',
        parameters=[common_params]
    )

    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            robot_state_publisher,
            simulation_motion_node,
            simulation_sensor_node,
            navigation_node,
            slam_debug_recorder_node,
            rviz_node,
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'record_slam_debug',
            default_value='true',
            description='Record /slam_debug messages to the 6DOF experiment CSV.'),
        gazebo_launch,
        delayed_nodes,
    ])
