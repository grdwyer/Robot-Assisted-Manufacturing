import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from math import pi


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # medport config file
    stock_config_file = get_package_share_directory('ram_tooling_support') + "/config/medpor_configuration.yaml"
    print(stock_config_file)
    stock_handler = Node(package='ram_tooling_support',
                         executable='stock_handler',
                         name='stock_handler',
                         output='log',
                         parameters=[stock_config_file]
                         )

    return LaunchDescription([stock_handler])
