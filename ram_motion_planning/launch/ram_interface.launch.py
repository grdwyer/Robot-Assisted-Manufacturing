import os
import yaml
from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def resolve_path(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


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


def load_xacro(package_name, file_path, mappings):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    doc = xacro.process_file(absolute_file_path, mappings=mappings).toprettyxml(indent='  ')
    return doc


def generate_launch_description():
    # Adding arguments
    declared_arguments = [DeclareLaunchArgument("planner", default_value="base",
                                                description="The toolpath planner to run (e.g. base, ompl, servo"),
                          DeclareLaunchArgument("robot_ip", default_value="192.170.10.2",
                                                description="IP address of the kuka KONI"),
                          DeclareLaunchArgument("robot_port", default_value="30200",
                                                description="Port used by the FRI (30200 - 30209"),
                          DeclareLaunchArgument("real_manipulator", default_value="false",
                                                description="Type of manipulator to startup (fake/false or real/true)"),
                          DeclareLaunchArgument("rviz", default_value="true", description="If rviz should run")]
    # specific arguments

    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    manipulator = LaunchConfiguration("real_manipulator")
    rviz = LaunchConfiguration("rviz")
    planner = LaunchConfiguration("planner")

    # Component yaml files are grouped in separate namespaces
    ######################
    #### Config Files ####
    ######################
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('ram_support'), "urdf", 'mock_iiwa_workcell.urdf.xacro']),
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
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_config = load_file('ram_moveit_config', 'config/iiwa_workcell.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('ram_moveit_config', 'config/kinematics.yaml')

    # Planning Functionality
    ompl_planning_pipeline_config = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        'start_state_max_bounds_error': 0.1}}
    ompl_planning_yaml = load_yaml('ram_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    nodes = []

    # RViz
    rviz_config_file = get_package_share_directory('ram_motion_planning') + "/launch/moveit_dev_setup.rviz"

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='own_log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 ompl_planning_pipeline_config,
                                 kinematics_yaml,
                                 ],
                     )
    nodes.append(rviz_node)

    rqt_node = Node(package='rqt_gui',
                    executable='rqt_gui',
                    name='rqt',
                    output='own_log',
                    )
    nodes.append(rqt_node)

    planner_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        get_package_share_directory('ram_motion_planning') + '/launch/toolpath_planner.launch.py'))

    return LaunchDescription(declared_arguments + nodes + [planner_launch])

