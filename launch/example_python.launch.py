import os
from typing import List
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def create_example_node(config:List[str]):

    # Python Node - Parameters
    example_node_parameters = {
        'example_string_param': LaunchConfiguration('example_string_param'),
        'example_double_param': LaunchConfiguration('example_double_param'),
        'example_vector_param': LaunchConfiguration('example_vector_param'),
        'example_bool_param':   LaunchConfiguration('example_bool_param'),
    }

    # Python Node + Parameters + YAML Config File
    example_node = Node(
        package='example_package', executable='example.py', name='example_node',
        output='screen', emulate_tty=True, arguments=[('__log_level:=debug')],
        parameters=[example_node_parameters] + config,
    )

    return example_node

def create_other_node(config:List[str]):

    # Topic Remappings
    remappings = [
        ('/original_topic', '/remapped_topic'),
        ('/cmd_vel', '/my_package/cmd_vel'),
    ]

    # Other Node + Remappings + YAML Config File
    other_node = Node(
        package='other_package', namespace='other_namespace', executable='other_executable', name='other_name',
        output='screen', emulate_tty=True, arguments=[('__log_level:=debug')],
        parameters=config, remappings=remappings,
    )

    return other_node

def generate_launch_description():

    # Launch Description
    launch_description = LaunchDescription()

    # Example - Arguments
    example_string_arg = DeclareLaunchArgument('example_string_param', default_value='Example')
    example_double_arg = DeclareLaunchArgument('example_double_param', default_value='0.15')
    example_vector_arg = DeclareLaunchArgument('example_vector_param', default_value='[3.14, 10.0]')
    example_bool_arg   = DeclareLaunchArgument('example_bool_param',   default_value='true')

    # Launch Description - Add Arguments
    launch_description.add_action(example_string_arg)
    launch_description.add_action(example_double_arg)
    launch_description.add_action(example_vector_arg)
    launch_description.add_action(example_bool_arg)

    # Include Other Launch Files
    other_package_dir = get_package_share_directory('example_package')
    included_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(other_package_dir + '/launch/global_parameter.launch.py'), launch_arguments={'example_global_param':'true'}.items())
    launch_description.add_action(included_launch)

    # Config File Path
    config = os.path.join(get_package_share_directory('example_package'), 'config','config.yaml')

    # Launch Description - Add Nodes
    launch_description.add_action(create_example_node([config]))
    # launch_description.add_action(create_other_node([config]))
    # launch_description.add_action(other_node)

    # Return Launch Description
    return launch_description
