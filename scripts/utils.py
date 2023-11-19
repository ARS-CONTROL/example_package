import os, shutil, rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_path

from std_msgs.msg import Float64MultiArray, MultiArrayDimension

#------------------------------------------------------- PATH UTILS ------------------------------------------------------#

def get_ros2_workspace_path(package_name:str) -> str:

    """ Get ROS2 Workspace Path Function """

    # Reverse Search for the 'build', 'install', 'log', 'src' directories
    current_path = os.path.dirname(os.path.abspath(__file__))
    while current_path != '/' and not all(dir in os.listdir(current_path) for dir in ['build', 'install', 'log', 'src']):
        current_path = os.path.dirname(current_path)

    # Get Path Working on String Slicing
    index = str(get_package_share_path(package_name)).find("install")
    new_path = os.path.dirname(os.path.abspath(__file__))[:index-1] if index != -1 else None

    # Assert Paths are Equal
    assert current_path == new_path, f"ROS2 Workspace Path Errors: {current_path} != {new_path}"
    return current_path

def get_package_path(package_name:str) -> str:

    """ Get Package Path Function """

    # Walk from ROS2 Workspace Path
    for root, dirs, files in os.walk(os.path.join(get_ros2_workspace_path(package_name), 'src')):

        if package_name in dirs and package_name_in_package_xml(package_name, os.path.join(root, package_name)):

            if not any(ignore_file in os.listdir(os.path.join(root, package_name)) for ignore_file in ['CATKIN_IGNORE', 'COLCON_IGNORE', 'AMENT_IGNORE']):
                return os.path.join(root, package_name)
            else: dirs.remove(package_name)

    assert False, f"Package Path Error: {package_name} not found in {os.path.join(get_ros2_workspace_path(), 'src')}"

def package_name_in_package_xml(package_name:str, package_path:str) -> bool:

    """ Check if Package Name is in Package XML File """

    import xml.etree.ElementTree as ET

    # Check if Package XML File Exists
    if not os.path.isfile(os.path.join(package_path, 'package.xml')): return False

    # Read Package XML File
    tree = ET.parse(os.path.join(package_path, 'package.xml'))

    # Assuming the <name> element is directly under the root
    if tree.getroot().find('name').text == package_name: return True

    return False

def delete_pycache_folders(project_dir:str):

    """ Delete Python `__pycache__` Folders Function """

    # Walk Through the Project Folders
    for root, dirs, files in os.walk(project_dir):

        if "__pycache__" in dirs:

            # Get `__pycache__` Path
            pycache_folder = os.path.join(root, "__pycache__")
            print(f"Deleting {pycache_folder}")

            # Delete `__pycache__`
            try: shutil.rmtree(pycache_folder)
            except Exception as e: print(f"An error occurred while deleting {pycache_folder}: {e}")

#----------------------------------------------------- SIGNAL HANDLER ----------------------------------------------------#

def custom_Signal_Handler(package_name):

    def signal_handler(sig, frame):

        ''' Custom SIG-INT Signal '''

        # Do Things ...

        # UR Stop Signal Handler
        ur_stop = UR_Signal_Handler()
        # signal.signal(signal.SIGINT, self.UR_Stop_Signal_Handler)

        # Delete Python `__pycache__` Folders
        delete_pycache_folders(get_package_path(package_name))

        # Shutdown ROS
        print('Stop Signal Received. Shutting down...')
        rclpy.shutdown()

    return signal_handler

class UR_Signal_Handler(Node):

    def __init__(self):

        super().__init__('ur_stop_node')

        # Create Publisher
        self.joint_group_vel_controller_publisher = self.create_publisher(Float64MultiArray, '/joint_group_vel_controller/command', 1)

        # Register the signal handler
        self.UR_Stop()

    def UR_Stop(self):

        ''' Stop SIG-INT Signal UR10e'''

        # Create Stop Message
        stop_vector = [0.0] * 6
        stop_msgs = Float64MultiArray(data=stop_vector)
        stop_msgs.layout.dim.append(MultiArrayDimension())
        stop_msgs.layout.dim[0].size = len(stop_vector)
        stop_msgs.layout.dim[0].stride = 1
        stop_msgs.layout.dim[0].label = 'velocity'

        # Publish Stop Message
        self.joint_group_vel_controller_publisher.publish(stop_msgs)

        # Shutdown ROS
        self.get_logger().info('Stop Signal Received. Stopping UR...')
        self.destroy_node()
