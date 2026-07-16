import os
import shutil
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, SetEnvironmentVariable


def find_free_gazebo_master_uri():
    existing_uri = os.environ.get('GAZEBO_MASTER_URI')
    if existing_uri:
        return existing_uri

    for port in range(11345, 11445):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.1)
            if sock.connect_ex(('127.0.0.1', port)) != 0:
                return f'http://127.0.0.1:{port}'

    return 'http://127.0.0.1:11345'


def generate_launch_description():
    package_share = get_package_share_directory('underwater_simulation')
    world_file = os.path.join(package_share, 'worlds', 'tunnel.world')
    model_path = os.path.join(package_share, 'models')
    gazebo_master_uri = find_free_gazebo_master_uri()

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = model_path
    if existing_model_path:
        gazebo_model_path = model_path + os.pathsep + existing_model_path

    actions = [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', gazebo_master_uri),
        LogInfo(msg=['GAZEBO_MODEL_PATH=', gazebo_model_path]),
        LogInfo(msg=['GAZEBO_MASTER_URI=', gazebo_master_uri]),
    ]

    gazebo = shutil.which('gazebo')
    if gazebo is None:
        actions.append(LogInfo(
            msg='gazebo executable not found in PATH. Install Gazebo classic or source the Gazebo setup, then rerun this launch file.'
        ))
    else:
        actions.append(ExecuteProcess(
            cmd=[
                gazebo,
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_file,
            ],
            additional_env={
                'GAZEBO_MODEL_PATH': gazebo_model_path,
                'GAZEBO_MASTER_URI': gazebo_master_uri,
            },
            output='screen',
        ))

    return LaunchDescription(actions)
