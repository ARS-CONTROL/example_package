#! /usr/bin/env python3

import rospy
import signal

from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from example_package.msg import ExampleMsg
from example_package.srv import ExampleSrv, ExampleSrvRequest, ExampleSrvResponse

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal

from dataclasses import dataclass
from typing import List

@dataclass
class example_struct:
    a: float
    b: str
    c: List[int]

#----------------------------------------------------- SIGNAL HANDLER ----------------------------------------------------#

def UR_Stop_Signal_Handler(sig, frame):

    ''' Stop SIG-INT Signal UR10e'''
    
    # Create Publisher
    joint_group_vel_controller_publisher = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
    stop_msgs = Float64MultiArray()

    # Create Stop Message
    stop_vector = [0.0] * 6
    stop_msgs.layout.dim.append(MultiArrayDimension())
    stop_msgs.layout.dim[0].size = len(stop_vector)
    stop_msgs.layout.dim[0].stride = 1
    stop_msgs.layout.dim[0].label = 'velocity'
    stop_msgs.data = stop_vector

    # Publish Stop Message
    joint_group_vel_controller_publisher.publish(stop_msgs)
    
    # Shutdown ROS
    rospy.signal_shutdown('Stop Signal Received')

def custom_Signal_Handler(sig, frame):
    
    ''' Custom SIG-INT Signal '''
    
    # Do Things ...
    
    # Shutdown ROS
    rospy.signal_shutdown('Custom Signal Received')

#------------------------------------------------------ CONSTRUCTOR ------------------------------------------------------#

class ExamplePackage:

    def __init__(self, ros_rate, example_data:str, example_vector:List[float]):
        
        # ---- ROS - RATE ---- #
        self.ros_rate_ = ros_rate

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
        
        # ---- LOAD GLOBAL PARAMETERS ---- #
        self.example_string_param_ = rospy.get_param('/example_string_param',  default='Default Example')
        self.example_double_param_ = rospy.get_param('/example_double_param_', default=1.1)

        # ---- LOAD NODE SPECIFIC PARAMETERS ---- #
        self.example_bool_param_   = rospy.get_param('/example_python_node/example_bool_param', default=True)
        self.example_vector_param_ = rospy.get_param('/example_cpp_node/example_vector_param',  default=[1.1, 1.1])

        # ---- LOAD NAMESPACED PARAMETERS ---- #
        self.example_string_param_ = rospy.get_param('/namespace_1/example_string_param', default='Default Example')
        self.example_double_param_ = rospy.get_param('/namespace_1/example_double_param', default=1.1)
        
        # ---- LOAD YAML FILE PARAMETERS ---- #
        self.example_string_param_ = rospy.get_param('/yaml_string_param', default='Default Example')
        self.example_double_param_ = rospy.get_param('/yaml_double_param', default=1.1)
        self.example_bool_param_   = rospy.get_param('/yaml_bool_param',   default=True)
        self.example_vector_param_ = rospy.get_param('/yaml_vector_param', default=[1.1, 1.1])
        
        # ---- ROS - PUBLISHERS ---- #
        self.example_publisher_ = rospy.Publisher('/global_example_publisher_topic_name', JointTrajectory, queue_size=1)
        self.example_custom_publisher_ = rospy.Publisher('/local_example_publisher_topic_name', ExampleMsg, queue_size=1)
        
        # ---- ROS - SUBSCRIBERS ---- #
        self.example_subscriber_ = rospy.Subscriber('/global_example_subscriber_topic_name', Float64MultiArray, callback=self.exampleSubscriberCallback)
        self.example_custom_subscriber_ = rospy.Subscriber('/group/example_subscriber_topic_name', ExampleMsg, self.exampleCustomSubscriberCallback)
       
        # ---- ROS - SERVICE CLIENTS ---- #
        self.example_client_ = rospy.ServiceProxy('/global_example_server_name', SetBool)
        self.example_custom_client_ = rospy.ServiceProxy('/group/example_server_name', ExampleSrv)

        # ---- ROS - SERVICE SERVERS ---- #
        self.example_server_ = rospy.Service('/global_example_server_name', SetBool, self.exampleServerCallback)
        self.example_custom_server_ = rospy.Service('/group/example_server_name', ExampleSrv, self.exampleCustomServerCallback)

        # ---- ROS - ACTIONS ---- #
        self.trajectory_client = actionlib.SimpleActionClient('/trajectory_publisher_action_name', FollowJointTrajectoryAction)

        # ---- DEBUG PRINT ---- #
        rospy.loginfo('Info Print')
        rospy.logwarn('Warn Print')
        rospy.logerr('Error Print')
        rospy.loginfo_once(f'Print Once: 10')
        rospy.loginfo_throttle(10, 'Print Every n Seconds')

#------------------------------------------------- SUBSCRIBER CALLBACKS --------------------------------------------------#

    def exampleSubscriberCallback(self, data:Float64MultiArray):
    
        received_msg = data
    
        # DO THINGS...


    def exampleCustomSubscriberCallback(self, data:ExampleMsg):
        
        received_custom_msg = data

        # DO THINGS...

#--------------------------------------------------- SERVER CALLBACKS ----------------------------------------------------#

    def exampleServerCallback(self, req:SetBoolRequest):
        
        # Received Request
        received_request = req
        
        # DO THINGS...

        # Response Filling
        response = SetBoolResponse()
        response.success = True

        return response

    def exampleCustomServerCallback(self, req:ExampleSrvRequest):
        
        # Received Request
        received_request = req
        string_value = received_request.string_value
        std_msgs_float = received_request.std_msgs_float
        # ...
        
        # DO THINGS...
        
        # Response Filling
        response = ExampleSrvResponse()
        response.int_value = 23
        response.success = True
        # ...

        return response

#------------------------------------------------ PUBLISH - CALL FUNCTIONS -----------------------------------------------#

    def PublishMessage(self):

        # ROS Message Creation
        trajectory_temp = JointTrajectory()

        # ROS Message Filling
        trajectory_temp.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        trajectory_temp.points.append(JointTrajectoryPoint())
        trajectory_temp.points[0].positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        trajectory_temp.points[0].time_from_start = rospy.Duration(5)

        # ROS Message Publication Publish Trajectory Position
        self.example_publisher_.publish(trajectory_temp)

    def CallService(self):

        # Service Creation
        request = SetBoolRequest()
        
        # Service Filling
        request.data = True

        # Wait For Service
        rospy.wait_for_service('/global_example_server_name')
        
        # Call Service
        response = self.example_client_(request)

    def CallAction(self):
    
        # Wait for the Action Server to Start
        rospy.loginfo('Waiting for Action Server to Start')
        self.trajectory_client.wait_for_server()
        rospy.loginfo('Action Server Started, Sending Goal')

        # ROS Action Goal Filling
        goal = FollowJointTrajectoryActionGoal()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        goal.trajectory.points.append(JointTrajectoryPoint())
        goal.trajectory.points[0].positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        goal.trajectory.points[0].time_from_start = rospy.Duration(5)

        # Send a Goal to the Action
        self.trajectory_client.send_goal(goal)
    
        # Wait for the Action to Return
        self.trajectory_client.wait_for_result()
    
        # Return the Result of Executing the Action
        return self.trajectory_client.get_result()

#---------------------------------------------------- UTILS FUNCTIONS ----------------------------------------------------#

    def spinner(self):
    
        # Publish a ROS Message
        self.PublishMessage()

        # Call a ROS Service Client
        self.CallService()

        # Call a ROS Action Client
        self.CallAction()

#--------------------------------------------------------- MAIN ---------------------------------------------------------#

if __name__ == '__main__':

    rospy.init_node('example_Node', anonymous=True, disable_signals=True)

    # Register Sig-Int Signal Handlers (CTRL+C)
    signal.signal(signal.SIGINT, UR_Stop_Signal_Handler)
    signal.signal(signal.SIGINT, custom_Signal_Handler)

    # ROS Initialization Parameters
    ros_rate = rospy.Rate(1000)

    # Class Initialization Parameters
    example_string = 'example_data'
    example_vector = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]

    # Initialize Class
    ep = ExamplePackage(ros_rate, example_string, example_vector)

    # Main Spinner Function
    while not rospy.is_shutdown(): ep.spinner()

    del ep
