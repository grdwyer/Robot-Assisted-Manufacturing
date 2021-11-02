import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
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
    declared_arguments = [DeclareLaunchArgument("robot_ip", default_value="192.170.10.2",
                                                description="IP address of the kuka KONI"),
                          DeclareLaunchArgument("robot_port", default_value="30200",
                                                description="Port used by the FRI (30200 - 30209"),
                          DeclareLaunchArgument("real_manipulator", default_value="false",
                                                description="Type of manipulator to startup (fake/false or real/true)"),
                          DeclareLaunchArgument("rviz", default_value="false", description="If rviz should run")]
    # specific arguments

    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    manipulator = LaunchConfiguration("real_manipulator")
    rviz = LaunchConfiguration("rviz")

    # Component yaml files are grouped in separate namespaces
    ######################
    #### Config Files ####
    ######################
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('ram_support'), "urdf", 'iiwa_workcell.urdf.xacro']),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            " ",
            "robot_port:=",
            robot_port,
            " ",
            " ",
            "hardware:=",
            manipulator,
            " ",
            " ",
            "blade_height:=",
            "0.0015",
            " "
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_description_semantic_config = load_file('ram_moveit_config', 'config/iiwa_workcell.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('ram_moveit_config', 'config/kinematics_kdl.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}
    print(robot_description_kinematics)

    # node_config = load_yaml("ram_motion_planning", "config/toolpath_follower_defaults.yaml")
    # print(node_config)

    toolpath_planner_params = os.path.join(
        get_package_share_directory('ram_motion_planning'),
        'config',
        'toolpath_follower_defaults.yaml'
    )
    nodes = []
    # Start the actual move_group interface node
    toolpath_planner = Node(name='toolpath_planner',
                            package='ram_motion_planning',
                            executable='base_toolpath_planner',
                            output='screen',
                            parameters=[toolpath_planner_params,
                                        robot_description,
                                        robot_description_semantic,
                                        robot_description_kinematics
                                        ])
    nodes.append(toolpath_planner)

    interface_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        get_package_share_directory('ram_motion_planning') + '/launch/ram_interface.launch.py'))
    nodes.append(interface_launch)

    return LaunchDescription(declared_arguments + nodes)
