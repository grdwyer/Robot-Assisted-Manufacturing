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

    implant_description_doc = load_xacro('ram_support', 'urdf/implant.urdf.xacro')

    robot_description_semantic_config = load_file('ram_moveit_config', 'config/iiwa_workcell.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('ram_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

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

    moveit_cpp_yaml_file_name = get_package_share_directory('ram_motion_planning') + "/config/moveit_cpp.yaml"
    nodes = []
    # Start the actual move_group node/action server
    run_moveit_cpp_node = Node(name='run_moveit_cpp',
                               package='ram_motion_planning',
                               executable='example_program',
                               output='screen',
                               parameters=[moveit_cpp_yaml_file_name,
                                           robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml,
                                           ompl_planning_pipeline_config,
                                           moveit_controllers])
    nodes.append(run_moveit_cpp_node)

    return LaunchDescription(nodes)
