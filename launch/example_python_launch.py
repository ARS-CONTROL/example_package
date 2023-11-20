import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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

    # Config File Path
    config = os.path.join(get_package_share_directory('example_package'), 'config','config.yaml')

    # Python Node + Parameters + YAML Config File
    example_node = Node(
        package='example_package', executable='example.py', name='example_node',
        output='screen', emulate_tty=True, arguments=[('__log_level:=debug')],
        parameters=[{
            'example_string_param': LaunchConfiguration('example_string_param'),
            'example_double_param': LaunchConfiguration('example_double_param'),
            'example_vector_param': LaunchConfiguration('example_vector_param'),
            'example_bool_param':   LaunchConfiguration('example_bool_param'),
        }] + [config],
    )

    # Other Node + Remappings + YAML Config File
    other_node = Node(
        package='other_package', namespace='other_namespace', executable='other_executable', name='other_name',
        output='screen', emulate_tty=True, arguments=[('__log_level:=debug')],
        parameters=[config],
        remappings=[
            ('/original_topic', '/remapped_topic'),
            ('/cmd_vel', '/my_package/cmd_vel'),
        ]
    )

    # Launch Description - Add Nodes
    launch_description.add_action(example_node)
    # launch_description.add_action(other_node)

    # Return Launch Description
    return launch_description
