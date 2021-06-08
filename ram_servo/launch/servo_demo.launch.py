import os
import yaml
from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Adding arguments
    declared_arguments = []
    # specific arguments
    declared_arguments.append(
        DeclareLaunchArgument("robot_ip", default_value="192.170.10.2",
                              description="IP address of the kuka KONI")
    )
    declared_arguments.append(
        DeclareLaunchArgument("robot_port", default_value="30200",
                              description="Port used by the FRI (30200 - 30209")
    )
    declared_arguments.append(
        DeclareLaunchArgument("real_manipulator", default_value="false", description="Type of manipulator to startup (fake/false or real/true)")
    )

    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    manipulator = LaunchConfiguration("real_manipulator")

    # print("\n\n\n\n\n\tManipulator set as {}\n\n\n\n".format(TextSubstitution().perform(manipulator)))

    # Component yaml files are grouped in separate namespaces
    ######################
    #### Config Files ####
    ######################
    # doc = load_xacro('ram_support', 'urdf/mock_iiwa_workcell.urdf.xacro', ['hardware:=false'])
    # robot_description = {'robot_description': doc}
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

    # Get parameters for the Pose Tracking node
    pose_tracking_yaml = load_yaml("ram_servo", "config/pose_tracking_settings.yaml")
    pose_tracking_params = {"moveit_servo": pose_tracking_yaml}

    # Get parameters for the Servo node
    servo_yaml = load_yaml(
        "ram_servo", "config/iiwa_simulated_config_pose_tracking.yaml"
    )
    servo_params = {"moveit_servo": servo_yaml}

    kinematics_yaml = load_yaml('ram_moveit_config', 'config/kinematics.yaml')

    nodes = []
    # RViz
    rviz_config_file = (
            get_package_share_directory("moveit_servo")
            + "/config/demo_rviz_pose_tracking.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # prefix=['xterm -e gdb -ex run --args'],
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
    )

    # nodes.append(rviz_node)

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    # nodes.append(robot_state_publisher)

    pose_tracking_node = Node(
        package="ram_servo",
        executable="servo_demo",
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

    # ros2_control using FakeSystem as hardware
    # Iiwa settings
    iiwa_controller = os.path.join(
        get_package_share_directory('ram_moveit_config'),
        'config',
        'ros_controllers.yaml'
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, iiwa_controller],
        prefix=['nice -n -20 '],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )
    # nodes.append(ros2_control_node)

    # Load controllers
    load_controllers = []
    for controller in ["iiwa_arm_controller", "joint_state_controller"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        declared_arguments +
        nodes
        # + load_controllers
    )