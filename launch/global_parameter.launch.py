import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Launch Description
    launch_description = LaunchDescription()

    # Global Config File Path
    global_parameters = os.path.join(get_package_share_directory('example_package'), 'config', 'global_config.yaml')

    # Global Parameter Server Node
    global_param_node = Node(package='example_package', executable='global_parameter_server', name='global_parameter_server',
        parameters=[global_parameters]
    )

    # Add Nodes to Launch Description
    launch_description.add_action(global_param_node)

    # Return Launch Description
    return launch_description
