import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    autoaim_dir = get_package_share_directory("webcamera_node")

    # Declare launch arguments
    start_as_component = DeclareLaunchArgument(
        'start_as_component',
        default_value='False',
        description='Start webcamera_node as a component (true) or as a standalone node (false)'
    )

    # Get configuration path
    config_path = os.path.join(
        autoaim_dir,
        'config',
        'camera_params.yaml'
    )

    # Create parameter file with substitutions
    param_rewrites = {}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=config_path,
            root_key='',
            param_rewrites=param_rewrites,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Get launch configuration values
    start_as_component_value = LaunchConfiguration('start_as_component')

    # Define ComposableNodeContainer
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='webcamera_node',
                plugin='camera::CameraNode',
                name='webcamera_node',
                parameters=[configured_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
        condition=IfCondition(start_as_component_value)
    )

    # Define standalone Node
    standalone_node = Node(
        package='webcamera_node',
        executable='webcamera_node_node',  
        name='webcamera_node',
        parameters=[configured_params],
        output='screen',
        condition=IfCondition(PythonExpression(['not ', start_as_component_value]))
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(start_as_component)

    # Add the actions to launch the camera node
    ld.add_action(container)
    ld.add_action(standalone_node)

    return ld