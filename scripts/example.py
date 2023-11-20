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

        super().__init__(node_name)

        # ---- ROS - RATE ---- #
        self.create_rate(ros_rate)

        # ---- Get Package Share Path ---- #
        share_path = get_package_share_path('example_package')
        print(f'Share Path: {share_path}')

        # ---- Get Resources Install Path ---- #
        resources_path = pkg_resources.resource_filename('example_package', 'resources')
        print(f'Resources Path: {resources_path}')

        # ---- Get Resources List ---- #
        resources_list = os.listdir(resources_path)
        print(f'Resources List: {resources_list}')

        # ---- Get ROS2 Package Path ---- #
        print(f'Package Path: {get_package_path("example_package")}')

        # ---- GLOBAL VARIABLES ---- #
        self.example_bool_   = False
        self.example_int_    = 100
        self.example_string_ = 'Default'
        self.example_vector_ = [1, 1.1, 0.0]
        self.example_struct_ = example_struct(1.0, 'String', [1,2,3])

        # ---- CLASS - PARAMETERS ---- #
        self.example_string_ = example_data
        self.example_vector_ = example_vector

        # ---- ROS - PARAMETERS ---- #
        self.declare_parameter('example_string_param',  'Default Example')
        self.declare_parameter('example_double_param_', 1.1)
        self.declare_parameter('example_bool_param',    False)
        self.declare_parameter('example_vector_param',  [1.1, 1.1])

        # ---- LOAD GLOBAL PARAMETERS ---- #
        # self.example_string_param_ = rospy.get_param('/example_string_param',  default='Default Example')
        # self.example_double_param_ = rospy.get_param('/example_double_param_', default=1.1)

        # ---- LOAD NODE SPECIFIC PARAMETERS ---- #
        # self.example_bool_param_   = rospy.get_param('/example_python_node/example_bool_param', default=True)
        # self.example_vector_param_ = rospy.get_param('/example_cpp_node/example_vector_param',  default=[1.1, 1.1])

        # ---- LOAD NAMESPACED PARAMETERS ---- #
        # self.example_string_param_ = rospy.get_param('/namespace_1/example_string_param', default='Default Example')
        # self.example_double_param_ = rospy.get_param('/namespace_1/example_double_param', default=1.1)

        # ---- LOAD YAML FILE PARAMETERS ---- #
        # self.example_string_param_ = rospy.get_param('/yaml_string_param', default='Default Example')
        # self.example_double_param_ = rospy.get_param('/yaml_double_param', default=1.1)
        # self.example_bool_param_   = rospy.get_param('/yaml_bool_param',   default=True)
        # self.example_vector_param_ = rospy.get_param('/yaml_vector_param', default=[1.1, 1.1])

        # ---- ROS - PUBLISHERS ---- #
        self.string_publisher_  = self.create_publisher(String, '/string_topic_name', 1)
        self.example_publisher_ = self.create_publisher(JointTrajectory, '/global_example_publisher_topic_name', 1)
        self.example_custom_publisher_ = self.create_publisher(ExampleMsg, '/local_example_publisher_topic_name', 1)

        # ---- ROS - SUBSCRIBERS ---- #
        self.string_subscriber_  = self.create_subscription(String, '/string_topic_name', self.stringSubscriberCallback, 1)
        self.example_subscriber_ = self.create_subscription(Float64MultiArray, '/global_example_subscriber_topic_name', self.exampleSubscriberCallback, 1)
        self.example_custom_subscriber_ = self.create_subscription(ExampleMsg, '/group/example_subscriber_topic_name', self.exampleCustomSubscriberCallback, 1)

        # ---- ROS - SERVICE CLIENTS ---- #
        self.example_client_ = self.create_client(SetBool, '/global_example_server_name')
        self.example_client_ = self.create_client(ExampleSrv, '/group/example_server_name')

        # ---- ROS - SERVICE SERVERS ---- #
        self.example_server_ = self.create_service(SetBool, '/global_example_server_name', self.exampleServerCallback)
        self.example_custom_server_ = self.create_service(ExampleSrv, '/group/example_server_name', self.exampleCustomServerCallback)

        # ---- ROS - ACTIONS ---- #
        self.trajectory_action_client_ = ActionClient(self, FollowJointTrajectory, '/trajectory_publisher_action_name')
        self.example_custom_action_client_ = ActionClient(self, ExampleAction, '/global_example_action_server_name')
        self.example_custom_action_server_ = ActionServer(self, ExampleAction, '/global_example_action_server_name', self.exampleActionCallback)

        # ---- DEBUG PRINT ---- #
        self.get_logger().info(f'Info Print')
        self.get_logger().warn(f'Warn Print')
        self.get_logger().error(f'Error Print')

#------------------------------------------------- SUBSCRIBER CALLBACKS --------------------------------------------------#

    def stringSubscriberCallback(self, data:String):

        received_msg = data
        # print(f'Received Message: {received_msg}')

        # DO THINGS...

    def exampleSubscriberCallback(self, data:Float64MultiArray):

        received_msg = data
        print(f'Received Message: {received_msg}')

        # DO THINGS...


    def exampleCustomSubscriberCallback(self, data:ExampleMsg):

        received_custom_msg = data
        print(f'Received Message: {received_custom_msg}')

        # DO THINGS...

#--------------------------------------------------- SERVER CALLBACKS ----------------------------------------------------#

    def exampleServerCallback(self, req:SetBool.Request, res:SetBool.Response):

        # Received Request
        received_request = req.data

        # DO THINGS...

        # Response Filling
        res.message = 'Example Message'
        res.success = True

        return res

    def exampleCustomServerCallback(self, req:ExampleSrv.Request, res:ExampleSrv.Response):

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

        # ROS Message Creation
        msg = String(data='Example String Message')

        # ROS String Message Publication
        if rclpy.ok(): self.string_publisher_.publish(msg)

    def PublishMessage(self):

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

        # Service Creation
        request = SetBool.Request(data=True)

        # Wait For Service
        self.example_client_.wait_for_service(timeout_sec=1.0)

        # Call Service
        future = self.example_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        # self.get_logger().info(f'Response: {response}')

    def CallAction(self):

        goal_msg = ExampleAction.Goal()
        goal_msg.example_request = 1

        # Wait for the Action Server to Start
        self.example_custom_action_client_.wait_for_server()

        # Send a Goal to the Action - Asynchronous
        return self.example_custom_action_client_.send_goal_async(goal_msg)

    def CallTrajectoryAction(self):

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

#---------------------------------------------------- UTILS FUNCTIONS ----------------------------------------------------#

    def spinner(self):

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

        # rclpy.spin_once(ep)
        # rclpy.spin(ep)

    # Delete Node and Shutdown ROS
    if ep: del ep

if __name__ == '__main__':

    main()
