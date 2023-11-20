#! /usr/bin/env python3

import sys, os, signal

# Import ROS2 Libraries
import rclpy, pkg_resources
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from ament_index_python.packages import get_package_share_path

# Import ROS Messages, Services, Actions
from builtin_interfaces.msg import Duration
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import SetBool
from rcl_interfaces.srv import GetParameters
from control_msgs.action import FollowJointTrajectory

# Import Custom ROS Messages, Services, Actions
from example_package.msg import ExampleMsg, ComposedMsg
from example_package.srv import ExampleSrv
from example_package.action import ExampleAction

# Import Utils Functions
from utils import get_package_path, custom_Signal_Handler

from dataclasses import dataclass
from typing import List

@dataclass
class example_struct:
    a: float
    b: str
    c: List[int]

#------------------------------------------------------ CONSTRUCTOR ------------------------------------------------------#

class ExamplePackage(Node):

    def __init__(self, node_name, ros_rate, example_data:str, example_vector:List[float]):

        # ---- Node Initialization ---- #
        super().__init__(node_name)

        # ---- Node Initialization with Automatic Parameters Declaration (Remove the self.declare_parameter Declarations) ---- #
        # super().__init__(node_name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # ---- ROS2 Rate ---- #
        self.create_rate(ros_rate)

        # ---- Get Package Share Path ---- #
        share_path = get_package_share_path('example_package')
        print(f'\n\nShare Path: {share_path}')

        # ---- Get Resources Install Path ---- #
        resources_path = pkg_resources.resource_filename('example_package', 'resources')
        print(f'Resources Path: {resources_path}')

        # ---- Get Resources List ---- #
        resources_list = os.listdir(resources_path)
        print(f'Resources List: {resources_list}')

        # ---- Get ROS2 Package Path ---- #
        print(f'Package Path: {get_package_path("example_package")}\n\n')

        # ---- Global Variables ---- #
        self.example_bool_   = False
        self.example_int_    = 100
        self.example_string_ = 'Default'
        self.example_vector_ = [1, 1.1, 0.0]
        self.example_struct_ = example_struct(1.0, 'String', [1,2,3])
        self.example_string_ = example_data
        self.example_vector_ = example_vector

        # ---- ROS2 - Parameters ---- #
        self.declare_parameter('example_string_param',  'Default Example')
        self.declare_parameter('example_double_param',  1.1)
        self.declare_parameter('example_vector_param',  [1.1, 1.1])
        self.declare_parameter('example_bool_param',    False)

        self.declare_parameter('yaml_string_param',     'Default YAML String')
        self.declare_parameter('yaml_double_param',     1.1)
        self.declare_parameter('yaml_bool_param',       False)
        self.declare_parameter('yaml_vector_param',     [1.1, 1.1])

        # ---- ROS2 - Use Parameters ---- #
        self.example_string_param = self.get_parameter('example_string_param').get_parameter_value().string_value
        self.example_double_param = self.get_parameter('example_double_param').get_parameter_value().double_value
        self.example_vector_param = self.get_parameter('example_vector_param').get_parameter_value().double_array_value
        self.example_bool_param   = self.get_parameter('example_bool_param').get_parameter_value().bool_value

        self.yaml_string_param    = self.get_parameter('yaml_string_param').get_parameter_value().string_value
        self.yaml_double_param    = self.get_parameter('yaml_double_param').get_parameter_value().double_value
        self.yaml_bool_param      = self.get_parameter('yaml_bool_param').get_parameter_value().bool_value
        self.yaml_vector_param    = self.get_parameter('yaml_vector_param').get_parameter_value().double_array_value

        self.get_logger().info(f'Example String Parameter: {self.example_string_param}')
        self.get_logger().info(f'Example Double Parameter: {self.example_double_param}')
        self.get_logger().info(f'Example Vector Parameter: {self.example_vector_param}')
        self.get_logger().info(f'Example Bool Parameter:   {self.example_bool_param}\n\n')

        self.get_logger().info(f'Example YAML String Parameter: {self.yaml_string_param}')
        self.get_logger().info(f'Example YAML Double Parameter: {self.yaml_double_param}')
        self.get_logger().info(f'Example YAML Bool Parameter:   {self.yaml_bool_param}')
        self.get_logger().info(f'Example YAML Vector Parameter: {self.yaml_vector_param}\n\n')

        # ---- ROS2 - Publishers ---- #
        self.string_publisher_  = self.create_publisher(String, '/string_topic_name', 1)
        self.example_publisher_ = self.create_publisher(JointTrajectory, '/global_example_publisher_topic_name', 1)
        self.example_custom_publisher_ = self.create_publisher(ExampleMsg, '/local_example_publisher_topic_name', 1)

        # ---- ROS2 - Subscribers ---- #
        self.string_subscriber_  = self.create_subscription(String, '/string_topic_name', self.stringSubscriberCallback, 1)
        self.example_subscriber_ = self.create_subscription(Float64MultiArray, '/global_example_subscriber_topic_name', self.exampleSubscriberCallback, 1)
        self.example_custom_subscriber_ = self.create_subscription(ExampleMsg, '/group/example_subscriber_topic_name', self.exampleCustomSubscriberCallback, 1)

        # ---- ROS2 - Service Clients ---- #
        self.example_client_ = self.create_client(SetBool, '/global_example_server_name')
        self.example_custom_client_ = self.create_client(ExampleSrv, '/group/example_server_name')
        self.global_parameters_client = self.create_client(GetParameters, '/global_parameter_server/get_parameters')

        # ---- ROS2 - Service Servers ---- #
        self.example_server_ = self.create_service(SetBool, '/global_example_server_name', self.exampleServerCallback)
        self.example_custom_server_ = self.create_service(ExampleSrv, '/group/example_server_name', self.exampleCustomServerCallback)

        # ---- ROS2 - Actions ---- #
        self.trajectory_action_client_ = ActionClient(self, FollowJointTrajectory, '/trajectory_publisher_action_name')
        self.example_custom_action_client_ = ActionClient(self, ExampleAction, '/global_example_action_server_name')
        self.example_custom_action_server_ = ActionServer(self, ExampleAction, '/global_example_action_server_name', self.exampleActionCallback)

        # ---- ROS2 - Get Global Parameters ---- #
        self.global_string_param, self.global_double_param = self.getGlobalParameters(['global_string_param', 'global_double_param'], ['str', 'double'])
        self.get_logger().info(f'Global String Parameter: {self.global_string_param}')
        self.get_logger().info(f'Global Double Parameter: {self.global_double_param}\n\n')

        # ---- ROS2 - Logger ---- #
        self.get_logger().info(f'Info Print')
        self.get_logger().warn(f'Warn Print')
        self.get_logger().error(f'Error Print')

#------------------------------------------------- SUBSCRIBER CALLBACKS --------------------------------------------------#

    def stringSubscriberCallback(self, data:String):

        """ Example Subscriber Callback """

        received_msg = data
        # print(f'Received Message: {received_msg}')

        # DO THINGS...

    def exampleSubscriberCallback(self, data:Float64MultiArray):

        """ Example Subscriber Callback """

        received_msg = data
        print(f'Received Message: {received_msg}')

        # DO THINGS...


    def exampleCustomSubscriberCallback(self, data:ExampleMsg):

        """ Example Custom Subscriber Callback """

        received_custom_msg = data
        print(f'Received Message: {received_custom_msg}')

        # DO THINGS...

#--------------------------------------------------- SERVER CALLBACKS ----------------------------------------------------#

    def exampleServerCallback(self, req:SetBool.Request, res:SetBool.Response):

        """ Example Server Callback """

        # Received Request
        received_request = req.data

        # DO THINGS...

        # Response Filling
        res.message = 'Example Message'
        res.success = True

        return res

    def exampleCustomServerCallback(self, req:ExampleSrv.Request, res:ExampleSrv.Response):

        """ Example Custom Server Callback """

        # Received Request
        received_request = req
        string_value = received_request.string_value
        std_msgs_float = received_request.std_msgs_float
        # ...

        # DO THINGS...

        # Response Filling
        res.int_value = 23
        res.success = True
        # ...

        return res

#--------------------------------------------------- ACTION CALLBACKS ----------------------------------------------------#

    def exampleActionCallback(self, goal_handle):

        """ Example Action Callback """

        # Received Goal
        # self.get_logger().info('Executing goal...')

        # DO THINGS...

        # Publish Feedback
        feedback = ExampleAction.Feedback(example_feedback=[1, 10])
        goal_handle.publish_feedback(feedback)

        # Goal Succeeded
        goal_handle.succeed()

        # Goal Result Filling
        result = ExampleAction.Result(example_result=[1, 10, 100])
        return result

#------------------------------------------------ PUBLISH - CALL FUNCTIONS -----------------------------------------------#

    def PublishStringMessage(self):

        """ Publish String Message """

        # ROS Message Creation
        msg = String(data='Example String Message')

        # ROS String Message Publication
        if rclpy.ok(): self.string_publisher_.publish(msg)

    def PublishMessage(self):

        """ Publish Message """

        # ROS Message Creation
        trajectory_temp = JointTrajectory()

        # ROS Message Filling
        trajectory_temp.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        trajectory_temp.points.append(JointTrajectoryPoint())
        trajectory_temp.points[0].positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        trajectory_temp.points[0].time_from_start = Duration(sec=5, nanosec=0)

        # ROS Message Publication
        if rclpy.ok(): self.example_publisher_.publish(trajectory_temp)

    def CallService(self):

        """ Call Service """

        # Service Creation
        request = SetBool.Request(data=True)

        # Wait For Service
        self.example_client_.wait_for_service(timeout_sec=1.0)

        # Call Service - Synchronous (Blocking - Critical)
        # response = self.example_client_.call(request)
        # self.get_logger().info(f'Response: {response}')

        # Call Service - Asynchronous
        future = self.example_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

    def CallAction(self):

        """ Call Action """

        goal_msg = ExampleAction.Goal()
        goal_msg.example_request = 1

        # Wait for the Action Server to Start
        self.example_custom_action_client_.wait_for_server()

        # Send a Goal to the Action - Asynchronous
        return self.example_custom_action_client_.send_goal_async(goal_msg)

    def CallTrajectoryAction(self):

        """ Call Trajectory Action """

        # Wait for the Action Server to Start
        self.get_logger().info('Waiting for Action Server to Start')
        self.trajectory_action_client_.wait_for_server()

        # ROS Action Goal Filling
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        goal.trajectory.points.append(JointTrajectoryPoint())
        goal.trajectory.points[0].positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        goal.trajectory.points[0].time_from_start = Duration(sec=5, nanosec=0)

        # Send a Goal to the Action - Synchronous (Blocking - Wait for Result)
        self.get_logger().info('Action Server Started, Sending Goal')
        result = self.trajectory_action_client_.send_goal(goal)

        # Return the Result of Executing the Action
        return result

    def getGlobalParameters(self, param_names:List[str], param_types:List[str]):

        """ Get Global Parameters """

        # Request Creation
        # request = GetParameters.Request(names=param_names)
        request = GetParameters.Request()
        request.names = param_names

        # Wait for the Service to Start
        self.global_parameters_client.wait_for_service()

        # Call Service - Asynchronous
        future = self.global_parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Unpack Response
        results = [result for result in future.result().values]

        # Get Parameters Values
        for i in range(len(results)):

            # Single Value
            if param_types[i] == 'str':      results[i] = results[i].string_value
            elif param_types[i] == 'int':    results[i] = results[i].integer_value
            elif param_types[i] == 'bool':   results[i] = results[i].bool_value
            elif param_types[i] == 'double': results[i] = results[i].double_value

            # Vector Value
            elif param_types[i] == 'str_vector':    results[i] = results[i].string_array_value
            elif param_types[i] == 'int_vector':    results[i] = results[i].integer_array_value
            elif param_types[i] == 'bool_vector':   results[i] = results[i].bool_array_value
            elif param_types[i] == 'double_vector': results[i] = results[i].double_array_value
            elif param_types[i] == 'byte_vector':   results[i] = results[i].byte_array_value

            # Invalid Type
            else: raise Exception(f'Invalid Parameter Type: {param_types[i]}')

        return results

#---------------------------------------------------- UTILS FUNCTIONS ----------------------------------------------------#

    def spinner(self):

        """ ROS2 - Spinner """

        # Publish a ROS String Message
        self.PublishStringMessage()

        # Publish a ROS Message
        self.PublishMessage()

        # Call a ROS Service Client
        self.CallService()

        # Call a ROS Action Client
        self.CallAction()

#--------------------------------------------------------- MAIN ---------------------------------------------------------#

def main(args=None):

    # Import Arguments
    if args is None: args = sys.argv

    # ROS Initialization
    rclpy.init(args=args)

    # Class Initialization Parameters
    example_string = 'example_data'
    example_vector = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]

    # Initialize Class
    ep = ExamplePackage('example_node', 1000, example_string, example_vector)

    # Register Sig-Int Signal Handlers (CTRL+C)
    signal.signal(signal.SIGINT, custom_Signal_Handler(package_name='example_package'))

    # Main Spinner Function
    while rclpy.ok():

        ep.spinner()

        rclpy.spin_once(ep, timeout_sec=0.001)
        # rclpy.spin(ep)

    # Delete Node and Shutdown ROS
    if ep: del ep

if __name__ == '__main__':

    main()
