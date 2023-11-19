#! /usr/bin/env python3

import sys, os, signal

# Import ROS2 Libraries
import rclpy, pkg_resources
from rclpy.node import Node
from ament_index_python.packages import get_package_share_path

# Import ROS Messages and Services
from builtin_interfaces.msg import Duration
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import SetBool

# from example_package.msg import ExampleMsg
# from example_package.srv import ExampleSrv, ExampleSrvRequest, ExampleSrvResponse

# import actionlib
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal

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
        self.example_string_param_ = ''
        self.example_double_param_ = 0.0
        self.example_bool_param_   = False
        self.example_vector_param_ = []
        
        # # ---- LOAD GLOBAL PARAMETERS ---- #
        # self.example_string_param_ = rospy.get_param('/example_string_param',  default='Default Example')
        # self.example_double_param_ = rospy.get_param('/example_double_param_', default=1.1)

        # # ---- LOAD NODE SPECIFIC PARAMETERS ---- #
        # self.example_bool_param_   = rospy.get_param('/example_python_node/example_bool_param', default=True)
        # self.example_vector_param_ = rospy.get_param('/example_cpp_node/example_vector_param',  default=[1.1, 1.1])

        # # ---- LOAD NAMESPACED PARAMETERS ---- #
        # self.example_string_param_ = rospy.get_param('/namespace_1/example_string_param', default='Default Example')
        # self.example_double_param_ = rospy.get_param('/namespace_1/example_double_param', default=1.1)
        
        # # ---- LOAD YAML FILE PARAMETERS ---- #
        # self.example_string_param_ = rospy.get_param('/yaml_string_param', default='Default Example')
        # self.example_double_param_ = rospy.get_param('/yaml_double_param', default=1.1)
        # self.example_bool_param_   = rospy.get_param('/yaml_bool_param',   default=True)
        # self.example_vector_param_ = rospy.get_param('/yaml_vector_param', default=[1.1, 1.1])
        
        # ---- ROS - PUBLISHERS ---- #
        self.string_publisher_  = self.create_publisher(String, '/string_topic_name', 1)
        self.example_publisher_ = self.create_publisher(JointTrajectory, '/global_example_publisher_topic_name', 1)
        # self.example_custom_publisher_ = rospy.Publisher('/local_example_publisher_topic_name', ExampleMsg, queue_size=1)
        
        # ---- ROS - SUBSCRIBERS ---- #
        self.string_subscriber_  = self.create_subscription(String, '/string_topic_name', self.stringSubscriberCallback, 1)
        self.example_subscriber_ = self.create_subscription(Float64MultiArray, '/global_example_subscriber_topic_name', self.exampleSubscriberCallback, 1)
        # self.example_custom_subscriber_ = rospy.Subscriber('/group/example_subscriber_topic_name', ExampleMsg, self.exampleCustomSubscriberCallback)
       
        # ---- ROS - SERVICE CLIENTS ---- #
        self.example_client_ = self.create_client(SetBool, '/global_example_server_name')
        # self.example_custom_client_ = rospy.ServiceProxy('/group/example_server_name', ExampleSrv)

        # # ---- ROS - SERVICE SERVERS ---- #
        self.example_server_ = self.create_service(SetBool, '/global_example_server_name', self.exampleServerCallback)
        # self.example_server_ = rospy.Service('/global_example_server_name', SetBool, self.exampleServerCallback)
        # self.example_custom_server_ = rospy.Service('/group/example_server_name', ExampleSrv, self.exampleCustomServerCallback)

        # # ---- ROS - ACTIONS ---- #
        # self.trajectory_client = actionlib.SimpleActionClient('/trajectory_publisher_action_name', FollowJointTrajectoryAction)

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


    # def exampleCustomSubscriberCallback(self, data:ExampleMsg):
        
    #     received_custom_msg = data

    #     # DO THINGS...

#--------------------------------------------------- SERVER CALLBACKS ----------------------------------------------------#

    def exampleServerCallback(self, req:SetBool.Request, res:SetBool.Response):

        # Received Request
        received_request = req.data

        # DO THINGS...

        # Response Filling
        res.message = 'Example Message'
        res.success = True

        return res

    # def exampleCustomServerCallback(self, req:ExampleSrvRequest):
        
    #     # Received Request
    #     received_request = req
    #     string_value = received_request.string_value
    #     std_msgs_float = received_request.std_msgs_float
    #     # ...
        
    #     # DO THINGS...
        
    #     # Response Filling
    #     response = ExampleSrvResponse()
    #     response.int_value = 23
    #     response.success = True
    #     # ...

    #     return response

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

    # def CallAction(self):
    
    #     # Wait for the Action Server to Start
    #     rospy.loginfo('Waiting for Action Server to Start')
    
    #     self.trajectory_client.wait_for_server()
    #     rospy.loginfo('Action Server Started, Sending Goal')

    #     # ROS Action Goal Filling
    #     goal = FollowJointTrajectoryActionGoal()
    #     goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    #     goal.trajectory.points.append(JointTrajectoryPoint())
    #     goal.trajectory.points[0].positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     goal.trajectory.points[0].time_from_start = rospy.Duration(5)

    #     # Send a Goal to the Action
    #     self.trajectory_client.send_goal(goal)
    
    #     # Wait for the Action to Return
    #     self.trajectory_client.wait_for_result()
    
    #     # Return the Result of Executing the Action
    #     return self.trajectory_client.get_result()

#---------------------------------------------------- UTILS FUNCTIONS ----------------------------------------------------#

    def spinner(self):

        # Publish a ROS String Message
        self.PublishStringMessage()

        # Publish a ROS Message
        self.PublishMessage()

        # Call a ROS Service Client
        self.CallService()

        # # Call a ROS Action Client
        # self.CallAction()

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
