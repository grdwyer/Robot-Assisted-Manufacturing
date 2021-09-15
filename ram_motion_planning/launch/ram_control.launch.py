import os
import yaml
from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
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
    declared_arguments = [DeclareLaunchArgument("robot_ip", default_value="192.170.10.2",
                                                description="IP address of the kuka KONI"),
                          DeclareLaunchArgument("robot_port", default_value="30200",
                                                description="Port used by the FRI (30200 - 30209"),
                          DeclareLaunchArgument("real_manipulator", default_value="true",
                                                description="Type of manipulator to startup (fake/false or real/true)"),
                          DeclareLaunchArgument("real_gripper", default_value="false",
                                                description="Type of gripper to startup (fake/false or real/true)"),
                          DeclareLaunchArgument("us_cutter", default_value="false",
                                                description="Load up the US cutter controller")]

    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    manipulator = LaunchConfiguration("real_manipulator")
    gripper = LaunchConfiguration("real_gripper")
    us_cutter = LaunchConfiguration("us_cutter")

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

    # Trajectory Execution Functionality
    controllers_yaml = load_yaml('ram_moveit_config', 'config/controllers.yaml')
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml,
                          'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}

    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                                         "publish_geometry_updates": True,
                                         "publish_state_updates": True,
                                         "publish_transforms_updates": True}

    nodes = []
    # Start the actual move_group node/action server
    move_group_node = Node(package='moveit_ros_move_group',
                           executable='move_group',
                           output='screen',
                           parameters=[robot_description,
                                       robot_description_semantic,
                                       kinematics_yaml,
                                       ompl_planning_pipeline_config,
                                       trajectory_execution,
                                       moveit_controllers,
                                       planning_scene_monitor_parameters
                                       ])
    nodes.append(move_group_node)

    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])
    nodes.append(robot_state_publisher)

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
    nodes.append(ros2_control_node)

    load_controllers = []
    for controller in ["iiwa_arm_controller", "joint_state_controller", "gripper_forward_command_controller_position"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]
    nodes += load_controllers

    # Gripper control
    sim_gripper_controller = Node(package="ram_gripper_control",
                                  executable="sim_gripper_controller",
                                  name="gripper_controller",
                                  output="screen",
                                  condition=UnlessCondition(gripper)
                                  )
    nodes.append(sim_gripper_controller)

    gripper_config = os.path.join(
        get_package_share_directory('ram_gripper_control'),
        'config',
        'gripper_config.yaml'
    )
    gripper_controller = Node(package="ram_gripper_control",
                              executable="gripper_controller",
                              name="gripper_controller",
                              output="screen",
                              parameters=[gripper_config],
                              condition=IfCondition(gripper)
                              )
    nodes.append(gripper_controller)

    server_environment = SetEnvironmentVariable(name='PIGPIO_ADDR',
                                                value='128.16.29.58')
    nodes.append(server_environment)

    daq_server = Node(package="daq_server",
                      executable="daq_server",
                      name="daq_server",
                      output="screen",
                      condition=IfCondition(gripper)
                      )
    nodes.append(daq_server)

    us_cutter_controller = Node(package="ram_tooling_support",
                                executable="us_cutter_controller",
                                name="us_cutter_controller",
                                output="screen",
                                condition=IfCondition(us_cutter)
                                )
    nodes.append(us_cutter_controller)

    # medpor config file
    stock_config_file = get_package_share_directory('ram_tooling_support') + "/config/medpor_configuration.yaml"
    # print(stock_config_file)
    stock_handler = Node(package='ram_tooling_support',
                         executable='stock_handler',
                         name='stock_handler',
                         output='log',
                         parameters=[stock_config_file]
                         )
    nodes.append(stock_handler)

    toolpath_handler = Node(package='ram_tooling_support',
                            executable='toolpath_handler',
                            name='toolpath_handler',
                            output='log',
                            )
    nodes.append(toolpath_handler)

    return LaunchDescription(declared_arguments + nodes)
