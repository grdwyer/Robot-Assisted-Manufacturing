import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_xacro(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    doc = xacro.process_file(absolute_file_path).toprettyxml(indent='  ')
    return doc


def generate_launch_description():
    # Component yaml files are grouped in separate namespaces
    ######################
    #### Config Files ####
    ######################
    doc = load_xacro('ram_support', 'urdf/iiwa_workcell.urdf.xacro')
    robot_description = {'robot_description': doc}

    robot_description_semantic_config = load_file('ram_moveit_config', 'config/iiwa_workcell.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('ram_moveit_config', 'config/kinematics.yaml')

    node_config = load_yaml("ram_motion_planning", "config/toolpath_follower_defaults.yaml")

    nodes = []
    # Start the actual move_group interface node
    toolpath_planner = Node(name='toolpath_planner',
                            package='ram_motion_planning',
                            executable='ompl_toolpath_planner',
                            output='screen',
                            parameters=[node_config,
                                        robot_description,
                                        robot_description_semantic,
                                        kinematics_yaml
                                        ])
    nodes.append(toolpath_planner)

    interface_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('ram_motion_planning') + '/launch/ram_interface.launch.py'))
    nodes.append(interface_launch)

    return LaunchDescription(nodes)
