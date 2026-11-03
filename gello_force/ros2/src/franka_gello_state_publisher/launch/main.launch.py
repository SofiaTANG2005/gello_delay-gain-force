"""Load force configuration into the independent publisher overlay."""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def nodes(context):
    path = os.path.join(get_package_share_directory('franka_gello_state_publisher'),
                        'config', LaunchConfiguration('config_file').perform(context))
    with open(path) as stream:
        configs = yaml.safe_load(stream)
    result = []
    for name, settings in configs.items():
        params = dict(settings)
        namespace = params.pop('namespace', '')
        params['gello_name'] = name
        params['com_port'] = os.path.join('/dev/serial/by-id', params['com_port'])
        result.append(Node(package='franka_gello_state_publisher', executable='gello_publisher',
                           name='gello_publisher', namespace=namespace, parameters=[params],
                           output='screen', on_exit=Shutdown()))
    return result


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='force_baselinef0.yaml'),
        OpaqueFunction(function=nodes)])
