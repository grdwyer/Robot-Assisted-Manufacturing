import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
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
    doc = load_xacro('ram_support', 'urdf/mock_iiwa_workcell.urdf.xacro')
    robot_description = {'robot_description': doc}

    robot_description_semantic_config = load_file('ram_moveit_config', 'config/iiwa_workcell.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('ram_moveit_config', 'config/kinematics.yaml')

    node_config = load_yaml("ram_motion_planning", "config/toolpath_follower_defaults.yaml")

    # Get parameters for the Pose Tracking node
    pose_tracking_yaml = load_yaml("ram_servo", "config/pose_tracking_settings.yaml")
    pose_tracking_params = {"moveit_servo": pose_tracking_yaml}

    # Get parameters for the Servo node
    servo_yaml = load_yaml(
        "ram_servo", "config/iiwa_simulated_config_pose_tracking.yaml"
    )
    servo_params = {"moveit_servo": servo_yaml}

    print(node_config)
    nodes = []
    # Start the actual move_group interface node
    toolpath_planner = Node(name='toolpath_planner',
                             package='ram_motion_planning',
                             executable='servo_toolpath_planner',
                             output='screen',
                             parameters=[node_config,
                                         robot_description,
                                         robot_description_semantic,
                                         kinematics_yaml
                                         ])
    nodes.append(toolpath_planner)

    pose_tracking_node = Node(
        package="ram_servo",
        executable="servo_subscriber",
        # prefix=['xterm -e gdb -ex run --args'],
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            pose_tracking_params,
            servo_params,
        ],
    )
    nodes.append(pose_tracking_node)

    return LaunchDescription(nodes)
